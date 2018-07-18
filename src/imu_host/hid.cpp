#include "hid.h"
#include <hidapi.h>

HID::HID()
{
    if (hid_init()) {
        throw HID_exception("Could not init HIDAPI");
    }
}

HID::~HID()
{
    hid_exit();
}

void HID::open(uint16_t pid, uint16_t vid) {
    if(dev_handle) {
        throw HID_exception("Already open");
    }

    dev_handle = hid_open(pid, vid, NULL);
    if(!dev_handle) {
        throw HID_exception("Could not open device");
    }
}

int HID::read(gsl::span<uint8_t> data) {
    if(!dev_handle) {
        throw HID_exception("Attempt an operation on a closed device");
    }
    int res = hid_read_timeout(dev_handle, &data[0], data.size(), 1000);
    if(res < 0) {
        throw HID_exception("Read error");
    }
    return res;
}

int HID::write(gsl::span<uint8_t> data) {
    if(!dev_handle) {
        throw HID_exception("Attempt an operation on a closed device");
    }
    int res = hid_write(dev_handle, &data[0], data.size());
    if(res < 0) {
        throw HID_exception("Write error");
    }
    return res;
}

int HID::set_feature(gsl::span<uint8_t> data) {
    if(!dev_handle) {
        throw HID_exception("Attempt an operation on a closed device");
    }
    int res = hid_send_feature_report(dev_handle, &data[0], data.size());
    if(res < 0) {
        throw HID_exception("Read error");
    }
    return res;
}

int HID::get_feature(gsl::span<uint8_t> data) {
    if(!dev_handle) {
        throw HID_exception("Attempt an operation on a closed device");
    }
    int res = hid_get_feature_report(dev_handle, &data[0], data.size());
    if(res < 0) {
        throw HID_exception("Write error");
    }
    return res;
}

void HID::set_nonblocking(bool is_nonblock) {
    if(!dev_handle) {
        throw HID_exception("Attempt an operation on a closed device");
    }
    hid_set_nonblocking(dev_handle, is_nonblock);
}

void HID::close(){
    if(!dev_handle) {
        throw HID_exception("Already closed");
    }
    hid_close(dev_handle);
}
