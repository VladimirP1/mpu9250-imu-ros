#pragma once
#include <string>
#include "gsl-lite.hpp"

using namespace std;

struct hid_device_;
typedef struct hid_device_ hid_device;

class HID_exception
{
    string desc;
public:
    HID_exception(string desc) : desc(desc) {}
    string get_desc(){ return desc; }
};

class HID
{
    hid_device *dev_handle = nullptr;
public:
    HID();
    ~HID();

    void open(uint16_t pid, uint16_t vid);
    int read(gsl::span<uint8_t> data);
    int write(gsl::span<uint8_t> data);
    int set_feature(gsl::span<uint8_t> data);
    int get_feature(gsl::span<uint8_t> data);
    void set_nonblocking(bool is_nonblock);
    void close();
};
