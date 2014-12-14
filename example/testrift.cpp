#include <iostream>

#include <boost/optional.hpp>

#include "libusbcpp.h"

    
struct Sample {
    uint64_t id;
    std::array<double, 3> accel; // m/s^2
    std::array<double, 3> gyro; // rad/s
};

struct MagSample {
    uint64_t id;
    std::array<double, 3> mag; // tesla
};

class Sink {
    boost::optional<uint64_t> last_id_;
public:
    void handle_sample(Sample const & sample) {
        if(last_id_ && sample.id != *last_id_ + 1) {
            std::cout << "dropped " << *last_id_ << " " << sample.id << std::endl;
        }
        last_id_ = sample.id;
        std::cout << sample.id << " " << sample.accel[0] << std::endl;
    }
    
    void handle_mag_sample(MagSample const & mag_sample) {
        //std::cout << mag_sample.id << " " << mag_sample.mag[0] << " " << mag_sample.mag[1] << " " << mag_sample.mag[2] << std::endl;
    }
};


class Tracker {
    Sink & sink_;
    boost::optional<uint64_t> last_sample_id_;
    boost::optional<std::array<double, 3> > last_mag_;

    std::array<double, 3> parse_vector(uint8_t const * data) {
        std::array<double, 3> res;
        struct {
            int32_t x: 21;
        } s;
        res[0] = 1e-4 * (s.x = (data[0] << 13) | (data[1] << 5) | ((data[2] & 0xF8) >> 3));
        res[1] = 1e-4 * (s.x = ((data[2] & 0x07) << 18) | (data[3] << 10) | (data[4] << 2) | ((data[5] & 0xC0) >> 6));
        res[2] = 1e-4 * (s.x = ((data[5] & 0x3F) << 15) | (data[6] << 7) | (data[7] >> 1));
        return res;
    }

    Sample parse_sample(uint64_t id, uint8_t const * data) {
        Sample res;
        res.id = id;
        res.accel = parse_vector(data);
        res.gyro = parse_vector(data + 8);
        return res;
    }

    int16_t u16_to_s16(uint16_t x) {
        if(x >= 32768) {
            return -static_cast<int16_t>(65535 - x) - 1;
        } else {
            return static_cast<int16_t>(x);
        }
    }

    void handle_report(uint8_t const * data) {
        uint8_t num_samples = data[3];
        uint8_t num_samples_here = std::min(uint8_t(2), num_samples);
        uint16_t running_sample_count = data[4] + 256 * data[5]; // is count of first sample in report
        //std::cout << static_cast<int>(num_samples) << " " << running_sample_count << std::endl;
        for(size_t i = 0; i < num_samples_here; i++) {
            uint16_t this_id = running_sample_count + i;
            uint64_t fixed;
            if(last_sample_id_) {
                fixed = this_id + (*last_sample_id_ & ~0xFFFF);
                while(fixed >= 65536 && fixed > *last_sample_id_ + 30000) {
                    fixed -= 65536;
                }
                while(fixed + 30000 < *last_sample_id_) {
                    fixed += 65536;
                }
            } else {
                fixed = this_id;
            }
            last_sample_id_ = fixed;
            sink_.handle_sample(parse_sample(fixed, data + 12 + 16 * i));
        }
        
        std::array<double, 3> this_mag;
        for(size_t i = 0; i < 3; i++) {
            this_mag[i] = u16_to_s16(data[44 + 2*i] + 256 * data[44 + 2*i + 1]) * 1e-4 * 1e-4;
        }
        
        if(!last_mag_ || this_mag != *last_mag_) {
            MagSample s;
            s.id = *last_sample_id_;
            s.mag = this_mag;
            sink_.handle_mag_sample(s);
            
            last_mag_ = this_mag;
        }
    }
public:
    Tracker(Sink & sink) :
        sink_(sink) {
    }
    
    void run() {
        libusbcpp::Context ctx;
        std::unique_ptr<libusbcpp::DeviceHandle> devp = ctx.open_device_with_vid_pid(0x2833, 0x0021);
        
        if(devp->kernel_driver_active(0)) {
            devp->detach_kernel_driver(0);
        }
        
        for(size_t i = 0; i < 3; i++) {
            devp->control_transfer(0x21, 9, 0x0302, 0, (unsigned char *)"\x02\x00\x00\x00\x00\x00\x00", 7);
        }
        devp->control_transfer(0x21, 9, 0x0311, 0, (unsigned char *)"\x11\x00\x00\x0b\x10\x27", 6);
        
        static size_t const TRANSFERS = 64;
        libusbcpp::Transfer transfers[TRANSFERS];
        uint8_t bufs[TRANSFERS][64];
        for(size_t i = 0; i < TRANSFERS; i++) {
            transfers[i].fill_interrupt(*devp, 0x81, bufs[i], 64, [this, i, &transfers, &bufs](libusb_transfer_status status, int actual_length) {
                if(status != LIBUSB_TRANSFER_COMPLETED) {
                    std::cout << "libusb transfer error " << status << std::endl;
                } else {
                    handle_report(bufs[i]);
                }
                transfers[i].submit();
            });
            transfers[i].submit();
        }
        
        while(true) {
            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 0.1 s
            ctx.handle_events_timeout_completed(tv);
        }
    }
};

int main() {
    Sink sink;
    Tracker tracker(sink);
    
    tracker.run();
    
    return EXIT_SUCCESS;
}
