#ifndef GUARD_XNYTIPRASSUKKTNM
#define GUARD_XNYTIPRASSUKKTNM

#include <cstdlib>
#include <iostream>
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


#define APPLY_EXCEPTIONS(F) \
    F(IO, IO) \
    F(InvalidParam, INVALID_PARAM) \
    F(Access, ACCESS) \
    F(NoDevice, NO_DEVICE) \
    F(NotFound, NOT_FOUND) \
    F(Busy, BUSY) \
    F(Timeout, TIMEOUT) \
    F(Overflow, OVERFLOW) \
    F(Pipe, PIPE) \
    F(Interrupted, INTERRUPTED) \
    F(NoMem, NO_MEM) \
    F(NotSupported, NOT_SUPPORTED) \
    F(Other, OTHER)

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
    };
APPLY_EXCEPTIONS(MAKE_EXC)
    

template<typename T>
T check_error(T res) {
    if(res >= 0) return res;
    #define HANDLE(NAME, CODE) if(res == LIBUSB_ERROR_##CODE) throw NAME##Error();
    APPLY_EXCEPTIONS(HANDLE)
    assert(false);
}

class Transfer : boost::noncopyable {
    libusb_transfer * transfer_;
    bool own_buffer_;
public:
    boost::function<void()> callback_;
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
        if(own_buffer_) {
            delete[] transfer_->buffer;
        }
        own_buffer_ = false;
        libusb_fill_control_transfer(transfer_, NULL, buffer, NULL, NULL, timeout);
    }
    void fill_control(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char const *data_out, uint16_t wLength, unsigned int timeout=0) {
        unsigned char * buffer = new unsigned char[LIBUSB_CONTROL_SETUP_SIZE + wLength];
        if(own_buffer_) {
            delete[] transfer_->buffer;
        }
        own_buffer_ = true;
        libusb_fill_control_setup(buffer, bmRequestType, bRequest, wValue, wIndex, wLength);
        bool out = (bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT;
        if(wLength) {
            assert(out == (data_out != NULL));
            if(out) {
                memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data_out, wLength);
            }
        }
        libusb_fill_control_transfer(transfer_, NULL, buffer, NULL, NULL, timeout);
    }
    void fill_bulk(unsigned char endpoint, unsigned char * buffer, int length, unsigned int timeout=0) {
        if(own_buffer_) {
            delete[] transfer_->buffer;
        }
        own_buffer_ = false;
        libusb_fill_bulk_transfer(transfer_, NULL, endpoint, buffer, length, NULL, NULL, timeout);
    }
    void fill_interrupt(unsigned char endpoint, unsigned char * buffer, int length, unsigned int timeout=0) {
        if(own_buffer_) {
            delete[] transfer_->buffer;
        }
        own_buffer_ = false;
        libusb_fill_interrupt_transfer(transfer_, NULL, endpoint, buffer, length, NULL, NULL, timeout);
    }
};

class Device;

class DeviceHandle : boost::noncopyable {
public:
    virtual ~DeviceHandle() { };
    virtual Device get_device() = 0;
    virtual bool kernel_driver_active(int interface_number) = 0;
    virtual void detach_kernel_driver(int interface_number) = 0;
    virtual int get_configuration() = 0;
    virtual void set_configuration(int configuration) = 0;
    virtual void claim_interface(int interface_number) = 0;
    virtual void release_interface(int interface_number) = 0;
    virtual void set_interface_alt_setting(int interface_number, int alternate_setting) = 0;
    virtual void submit_sync(Transfer & transfer) = 0;
    virtual boost::function<void()> submit_async(Transfer & transfer, boost::function<void()> callback) = 0; // returns cancellation function
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
    
    Device get_device() override;
    
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
    
    void submit_sync(Transfer & transfer) override {
        int completed = 0;
        submit_async(transfer, [&completed]() {
            completed = 1;
        });
        while(!completed) {
            check_error(libusb_handle_events_completed(context_, &completed));
        }
        if(transfer.get_transfer().status != LIBUSB_TRANSFER_COMPLETED) {
            throw std::runtime_error("transfer failed");
        }
    }
    
private:
    static void cb(libusb_transfer *transfer) {
        Transfer & t = *reinterpret_cast<Transfer *>(transfer->user_data);
        try {
            t.callback_();
        } catch(std::exception const & exc) {
            std::cout << "caught in callback: " << exc.what() << std::endl;
            std::terminate();
        } catch(...) {
            std::cout << "caught in callback: ???" << std::endl;
            std::terminate();
        }
    }
public:
    boost::function<void()> submit_async(Transfer & transfer, boost::function<void()> callback) override {
        transfer.get_transfer().dev_handle = handle_;
        transfer.get_transfer().callback = cb;
        transfer.get_transfer().user_data = &transfer;
        transfer.callback_ = callback;
        check_error(libusb_submit_transfer(&transfer.get_transfer()));
        return [&transfer]() {
            check_error(libusb_cancel_transfer(&transfer.get_transfer()));
        };
    }
};

class Device {
    libusb_context * context_;
    libusb_device * device_;
    std::vector<std::unique_ptr<libusb_config_descriptor, decltype(&libusb_free_config_descriptor)>> config_descriptors_;
public:
    Device(libusb_context * context, libusb_device * device) : context_(context), device_(device) {
        libusb_ref_device(device_);
        for(int i = 0; i < get_device_descriptor().bNumConfigurations; i++) {
            libusb_config_descriptor * p;
            check_error(libusb_get_config_descriptor(device_, i, &p));
            config_descriptors_.emplace_back(p, libusb_free_config_descriptor);
        }
    }
    Device(Device const & device) : context_(device.context_), device_(device.device_) {
        libusb_ref_device(device_);
    }
    Device(Device && device) : context_(device.context_), device_(device.device_) {
        libusb_ref_device(device_);
    }
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
    
    uint8_t get_bus_number() const { return libusb_get_bus_number(device_); }
    uint8_t get_port_number() const { return libusb_get_port_number(device_); }
    int get_port_numbers(uint8_t * port_numbers, int port_numbers_len) const { return check_error(libusb_get_port_numbers(device_, port_numbers, port_numbers_len)); }
    // get_parent
    uint8_t get_device_address() const { return libusb_get_device_address(device_); }
    libusb_speed get_device_speed() const { return libusb_speed(libusb_get_device_speed(device_)); }
    uint16_t get_max_packet_size(unsigned char endpoint) const { return check_error(libusb_get_max_packet_size(device_, endpoint)); }
    int get_max_iso_packet_size(unsigned char endpoint) const { return check_error(libusb_get_max_iso_packet_size(device_, endpoint)); }
};

inline Device LibUSBDeviceHandle::get_device() {
    return Device(context_, libusb_get_device(handle_));
}

class Context : boost::noncopyable {
public:
    virtual ~Context() { };
    virtual std::vector<Device> get_device_list() = 0;
    
    std::unique_ptr<DeviceHandle> open_device_with_vid_pid(uint16_t vendor_id, uint16_t product_id) {
        BOOST_FOREACH(Device const & d, get_device_list()) {
            libusb_device_descriptor desc = d.get_device_descriptor();
            if(desc.idVendor == vendor_id && desc.idProduct == product_id) {
                return d.open();
            }
        }
        throw std::runtime_error("device not found");
    }
    
    virtual void handle_events_timeout_completed(timeval & tv, int * completed=nullptr) = 0;
};

class LibUSBContext : public Context {
    libusb_context * context_;
public:
    LibUSBContext() {
        check_error(libusb_init(&context_));
    }
    ~LibUSBContext() {
        libusb_exit(context_);
    }
    
    void set_debug(libusb_log_level level) {
        libusb_set_debug(context_, level);
    }
    
    std::vector<Device> get_device_list() override {
        libusb_device **list;
        ssize_t count = check_error(libusb_get_device_list(context_, &list));
        std::vector<Device> result;
        for(ssize_t i = 0; i < count; i++) {
            result.emplace_back(context_, list[i]);
        }
        libusb_free_device_list(list, true);
        return result;
    }
    
    void handle_events_timeout_completed(timeval & tv, int * completed=nullptr) override {
        check_error(libusb_handle_events_timeout_completed(context_, &tv, completed));
    }
};


}

#endif
