#ifndef HID_MPU_H
#define HID_MPU_H
#include "accel_solver.h"
#include "hid.h"
#include <fstream>
#include <queue>
#include <array>

#define ACC_TO_G (4.0 / 32768.0)
#define GYR_TO_DEGS (2000.0 / 32768.0)

struct MPU_sample {
  arma::vec acc_g;
  arma::vec gyr_degs;
  uint64_t time_us;
};

struct MPU_calibration {
    arma::vec accel_cal {
        0,
        0,
        0,
        1,
        1,
        1
    };
    arma::vec gyro_cal {0, 0, 0};
};

class MPU : public HID
{
    MPU_calibration calibration;
    std::queue<MPU_sample> samples;
    std::array<uint8_t, 16> zeros;
    uint32_t time_last = 0;
    uint64_t time_correction = 0;

    void read_mpu();
    
protected:
    void decode_at(uint8_t * ptr);
    int16_t decode_short_at(uint8_t* ptr);
    uint32_t decode_uword_at(uint8_t* ptr);
    MPU_sample transform_by_calibration(MPU_sample s);
    uint64_t fix_time(uint32_t in);

public:
    MPU();
    MPU_sample get_next_sample();
    MPU_sample get_next_sample_cal();
    arma::vec calibrate_acc(int samples = 2000);
    arma::vec calibrate_gyr(int samples = 2000);
    bool load_calibration(string filename);
    bool save_calibration(string filename);
};

#endif // HID_MPU_H
