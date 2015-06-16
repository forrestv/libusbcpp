#ifndef GUARD_XNYTIPRASSUKKTNM
#define GUARD_XNYTIPRASSUKKTNM

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
static std::unordered_map<int, std::function<void(void)> > errors;
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
    static ErrorInserter<NAME##Error> _##NAME##Error_inserter(LIBUSB_ERROR_##CODE);
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

class Transfer : boost::noncopyable {
    libusb_transfer * transfer_;
    bool own_buffer_;
public:
    Transfer(int iso_packets=0) {
        transfer_ = libusb_alloc_transfer(iso_packets);
        if(!transfer_) {
            throw std::bad_alloc();
        }
        own_buffer_ = false;
    }
    ~Transfer() {
        if(own_buffer_) {
            delete[] transfer_->buffer;
        }
        libusb_free_transfer(transfer_);
    }
    
    libusb_transfer & get_transfer() {
        return *transfer_;
    }
    
    void fill_control(unsigned char * buffer, unsigned int timeout=0) {
        libusb_fill_control_transfer(transfer_, NULL, buffer, NULL, NULL, timeout);
    }
    void fill_control(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char *data_out, uint16_t wLength, unsigned int timeout=0) {
        unsigned char * buffer = new unsigned char[LIBUSB_CONTROL_SETUP_SIZE + wLength];
        assert(!own_buffer_);
        own_buffer_ = true;
        libusb_fill_control_setup(buffer, bmRequestType, bRequest, wValue, wIndex, wLength);
        bool out = (bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT;
        if(wLength) {
            assert(out == (data_out != NULL));
            if(out) {
                memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data_out, wLength);
            }
        }
        fill_control(buffer, timeout);
    }
    void fill_bulk(unsigned char endpoint, unsigned char * buffer, int length, unsigned int timeout=0) {
        libusb_fill_bulk_transfer(transfer_, NULL, endpoint, buffer, length, NULL, NULL, timeout);
    }
    void fill_interrupt(unsigned char endpoint, unsigned char * buffer, int length, unsigned int timeout=0) {
        libusb_fill_interrupt_transfer(transfer_, NULL, endpoint, buffer, length, NULL, NULL, timeout);
    }
};

class DeviceHandle : boost::noncopyable {
public:
    virtual ~DeviceHandle() { };
    virtual bool kernel_driver_active(int interface_number) = 0;
    virtual void detach_kernel_driver(int interface_number) = 0;
    virtual int get_configuration() = 0;
    virtual void set_configuration(int configuration) = 0;
    virtual void claim_interface(int interface_number) = 0;
    virtual void release_interface(int interface_number) = 0;
    virtual void set_interface_alt_setting(int interface_number, int alternate_setting) = 0;
    virtual void submit_sync(Transfer & transfer) = 0;
    virtual void submit_async(Transfer & transfer, boost::function<void()> callback) = 0;
};

class LibUSBDeviceHandle : public DeviceHandle {
    libusb_context * context_;
    libusb_device_handle * handle_;
public:
    LibUSBDeviceHandle(libusb_context * context, libusb_device * dev) : context_(context) {
        check_error(libusb_open(dev, &handle_));
    }
    ~LibUSBDeviceHandle() override {
        libusb_close(handle_);
    }
    
    bool kernel_driver_active(int interface_number) override {
        return check_error(libusb_kernel_driver_active(handle_, interface_number)) == 1;
    }
    void detach_kernel_driver(int interface_number) override {
        check_error(libusb_detach_kernel_driver(handle_, interface_number));
    }
    
    int get_configuration() override {
        int res;
        check_error(libusb_get_configuration(handle_, &res));
        return res;
    }
    void set_configuration(int configuration) override {
        check_error(libusb_set_configuration(handle_, configuration));
    }
    
    void claim_interface(int interface_number) override {
        check_error(libusb_claim_interface(handle_, interface_number));
    }
    void release_interface(int interface_number) override {
        check_error(libusb_release_interface(handle_, interface_number));
    }
    void set_interface_alt_setting(int interface_number, int alternate_setting) override {
        check_error(libusb_set_interface_alt_setting(handle_, interface_number, alternate_setting));
    }
    
    void submit_sync(Transfer & transfer) {
        int completed = 0;
        submit_async(transfer, [&completed]() {
            completed = 1;
        });
        while(!completed) {
            check_error(libusb_handle_events_completed(context_, &completed));
        }
    }
    
private:
    class CallbackHelper {
    public:
        CallbackHelper(boost::function<void()> callback) : callback_(callback) { };
        boost::function<void()> callback_;
    };
    static void cb(libusb_transfer *transfer) {
        try {
            CallbackHelper *tp = reinterpret_cast<CallbackHelper *>(transfer->user_data);
            tp->callback_();
            delete tp;
        } catch(std::exception const & exc) {
            std::cout << "caught in callback: " << exc.what() << std::endl;
        } catch(...) {
            std::cout << "caught in callback: ???" << std::endl;
        }
    }
public:
    void submit_async(Transfer & transfer, boost::function<void()> callback) {
        transfer.get_transfer().dev_handle = handle_;
        transfer.get_transfer().callback = cb;
        transfer.get_transfer().user_data = new CallbackHelper(callback);
        check_error(libusb_submit_transfer(&transfer.get_transfer()));
    }
};

class Device {
    libusb_context * context_;
    libusb_device * device_;
public:
    Device(libusb_context * context, libusb_device * device) : context_(context), device_(device) {
        libusb_ref_device(device_);
    }
    Device(Device const & device) : context_(device.context_), device_(device.device_) {
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
        return std::unique_ptr<DeviceHandle>(new LibUSBDeviceHandle(context_, device_));
    }
};

class Context : boost::noncopyable {
    libusb_context * context_;
public:
    Context(int debug_level=LIBUSB_LOG_LEVEL_NONE) {
        check_error(libusb_init(&context_));
        libusb_set_debug(context_, debug_level);
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
            result.emplace_back(context_, list[i]);
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

#endif
