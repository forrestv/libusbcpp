#include <cstdlib>
#include <stdexcept>
#include <sstream>
#include <unordered_map>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/utility.hpp>

#include "libusb.h"

namespace libusbcpp {


class Error : public std::exception {
public:
    virtual char const * what() const throw () override = 0;
};
std::unordered_map<int, std::function<void(void)> > errors;
template<typename ErrorType>
struct ErrorInserter {
    ErrorInserter(int error_code) {
        errors.insert(std::make_pair(error_code, []() {
            throw ErrorType();
        }));
    }
};
#define MAKE_EXC(NAME, CODE) \
    class NAME##Error : public Error { \
        std::string message_; \
    public: \
        NAME##Error() { \
            message_ = std::string("libusb error: ") + libusb_strerror(LIBUSB_ERROR_##CODE); \
        } \
        char const * what() const throw () override { \
            return message_.c_str(); \
        } \
    }; \
    ErrorInserter<NAME##Error> _##NAME##Error_inserter(LIBUSB_ERROR_##CODE);
MAKE_EXC(IO, IO)
MAKE_EXC(InvalidParam, INVALID_PARAM)
MAKE_EXC(Access, ACCESS)
MAKE_EXC(NoDevice, NO_DEVICE)
MAKE_EXC(NotFound, NOT_FOUND)
MAKE_EXC(Busy, BUSY)
MAKE_EXC(Timeout, TIMEOUT)
MAKE_EXC(Overflow, OVERFLOW)
MAKE_EXC(Pipe, PIPE)
MAKE_EXC(Interrupted, INTERRUPTED)
MAKE_EXC(NoMem, NO_MEM)
MAKE_EXC(NotSupported, NOT_SUPPORTED)
MAKE_EXC(Other, OTHER)

template<typename T>
T check_error(T res) {
    if(res >= 0) return res;
    errors[res](); // will always throw
    assert(false);
}

class DeviceHandle : boost::noncopyable {
public:
    libusb_device_handle * handle_; // XXX shouldn't be public, but Transfer needs it
    DeviceHandle(libusb_device * dev) {
        check_error(libusb_open(dev, &handle_));
    }
    ~DeviceHandle() {
        libusb_close(handle_);
    }
    
    bool kernel_driver_active(int interface_number) {
        return check_error(libusb_kernel_driver_active(handle_, interface_number)) == 1;
    }
    void detach_kernel_driver(int interface_number) {
        check_error(libusb_detach_kernel_driver(handle_, interface_number));
    }
    
    int get_configuration() {
        int res;
        check_error(libusb_get_configuration(handle_, &res));
        return res;
    }
    void set_configuration(int configuration) {
        check_error(libusb_set_configuration(handle_, configuration));
    }
    
    void claim_interface(int interface_number) {
        check_error(libusb_claim_interface(handle_, interface_number));
    }
    void release_interface(int interface_number) {
        check_error(libusb_release_interface(handle_, interface_number));
    }
    void set_interface_alt_setting(int interface_number, int alternate_setting) {
        check_error(libusb_set_interface_alt_setting(handle_, interface_number, alternate_setting));
    }
    
    size_t control_transfer(uint8_t bmRequestType, uint8_t bRequest,
    uint16_t wValue, uint16_t wIndex, unsigned char * data, uint16_t wLength,
    unsigned int timeout=0) {
        return check_error(libusb_control_transfer(handle_, bmRequestType,
            bRequest, wValue, wIndex, data, wLength, timeout));
    }
    
    void interrupt_transfer(unsigned char endpoint, unsigned char * data,
    int length, int & transferred, unsigned int timeout=0) {
        check_error(libusb_interrupt_transfer(handle_, endpoint, data,
            length, &transferred, timeout));
    }
};

class Transfer : boost::noncopyable {
    static void cb(libusb_transfer *transfer) {
        try {
            Transfer & t = *reinterpret_cast<Transfer *>(transfer->user_data);
            t.active_ = false;
            t.callback_(t.transfer_->status, t.transfer_->actual_length);
        } catch(std::exception const & exc) {
            std::cout << "caught in callback: " << exc.what() << std::endl;
        } catch(...) {
            std::cout << "caught in callback: ???" << std::endl;
        }
    }
    
    libusb_transfer * transfer_;
    boost::function<void(libusb_transfer_status, int)> callback_;
    bool active_;
public:
    Transfer(int iso_packets=0) {
        transfer_ = libusb_alloc_transfer(iso_packets);
        if(!transfer_) {
            throw std::bad_alloc();
        }
        active_ = false;
    }
    ~Transfer() {
        if(!active_) {
            // leak the transfer if we can't free it. ):
            // alternative solution would be to cancel and then somehow wait
            // for callback to confirm cancel
            libusb_free_transfer(transfer_);
        }
    }
    
    void submit() {
        check_error(libusb_submit_transfer(transfer_));
        active_ = true;
    }
    void cancel() {
        check_error(libusb_cancel_transfer(transfer_));
    }
    
    void fill_bulk(DeviceHandle & dev_handle, unsigned char endpoint, unsigned char * buffer, int length, boost::function<void(libusb_transfer_status, int)> callback, unsigned int timeout=0) {
        callback_ = callback;
        libusb_fill_bulk_transfer(transfer_, dev_handle.handle_, endpoint, buffer, length, cb, reinterpret_cast<void *>(this), timeout);
    }
    void fill_interrupt(DeviceHandle & dev_handle, unsigned char endpoint, unsigned char * buffer, int length, boost::function<void(libusb_transfer_status, int)> callback, unsigned int timeout=0) {
        callback_ = callback;
        libusb_fill_interrupt_transfer(transfer_, dev_handle.handle_, endpoint, buffer, length, cb, reinterpret_cast<void *>(this), timeout);
    }
};

class Device {
    libusb_device * device_;
public:
    Device(libusb_device * device) : device_(device) {
        libusb_ref_device(device_);
    }
    Device(Device const & device) : device_(device.device_) {
        libusb_ref_device(device_);
    }
    Device(Device && device) = delete;
    ~Device() {
        libusb_unref_device(device_);
    }
    Device & operator=(Device const & other) = delete;
    Device & operator=(Device && other) = delete;
    
    libusb_device_descriptor get_device_descriptor() const {
        libusb_device_descriptor res;
        check_error(libusb_get_device_descriptor(device_, &res));
        return res;
    }
    
    std::unique_ptr<DeviceHandle> open() const {
        return std::unique_ptr<DeviceHandle>(new DeviceHandle(device_));
    }
};

class Context : boost::noncopyable {
    libusb_context * context_;
public:
    Context() {
        check_error(libusb_init(&context_));
    }
    ~Context() {
        libusb_exit(context_);
    }
    
    void set_debug(libusb_log_level level) {
        libusb_set_debug(context_, level);
    }
    
    std::vector<Device> get_device_list() {
        libusb_device **list;
        ssize_t count = check_error(libusb_get_device_list(context_, &list));
        std::vector<Device> result;
        for(ssize_t i = 0; i < count; i++) {
            result.emplace_back(list[i]);
        }
        libusb_free_device_list(list, true);
        return result;
    }
    
    std::unique_ptr<DeviceHandle> open_device_with_vid_pid(uint16_t vendor_id, uint16_t product_id) {
        BOOST_FOREACH(Device const & d, get_device_list()) {
            libusb_device_descriptor desc = d.get_device_descriptor();
            if(desc.idVendor == vendor_id && desc.idProduct == product_id) {
                return d.open();
            }
        }
        throw std::runtime_error("device not found");
    }
    
    void handle_events_timeout_completed(timeval & tv, int * completed=nullptr) {
        check_error(libusb_handle_events_timeout_completed(context_, &tv, completed));
    }
};


}
