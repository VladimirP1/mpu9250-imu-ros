#include "hid_mpu.h"
#include <iostream>

MPU::MPU(){
    std::fill(zeros.begin(), zeros.end(), 0);
}

int16_t MPU::decode_short_at(uint8_t* ptr) {
    return ptr[0] << 8 | ptr[1];
}

uint32_t MPU::decode_uword_at(uint8_t* ptr) {
    return ptr[0] << 24 | ptr[1] << 16 | ptr[2] << 8 | ptr[3];
}

uint64_t MPU::fix_time(uint32_t in) {
    if(in < time_last) {
        time_correction += 0x100000000ULL;
    }
    time_last = in;
    return in + time_correction;
}

void MPU::decode_at(uint8_t* ptr) {
    bool valid = !std::equal(ptr, ptr + 16, zeros.begin());
    if(!valid) return;
    MPU_sample sample =
    {
        .acc_g = {
            decode_short_at(ptr)      * ACC_TO_G,
            decode_short_at(ptr + 2)  * ACC_TO_G,
            decode_short_at(ptr + 4)  * ACC_TO_G
        },
        .gyr_degs = {
            decode_short_at(ptr + 6)  * GYR_TO_DEGS,
            decode_short_at(ptr + 8)  * GYR_TO_DEGS,
            decode_short_at(ptr + 10) * GYR_TO_DEGS
        },
        .time_us    = fix_time(decode_uword_at(ptr + 12)) / 120
    };
    samples.push(sample);
}

void MPU::read_mpu(){
    std::array<uint8_t, 32> buf;
    int res = read(buf);
    if (res != 32) {
        cerr << res << endl;
        throw HID_exception("Bad sample length.");
    }
    decode_at(buf.begin());
    decode_at(buf.begin() + 16);
}

MPU_sample MPU::get_next_sample() {
    int timeout = 0;
    while(samples.empty()) {
        read_mpu();
        if(timeout > 100){
            throw HID_exception("Device sends invalid reports.");
        }
        timeout++;
    }
    MPU_sample ret = samples.front();
    samples.pop();
    return ret;
}

arma::vec MPU::calibrate_acc(int samples) {
    string dummy;
    MPU_sample sample;
    arma::vec accumulated(3);
    vector<arma::vec> measurements;
    while(1) {
        cout << "Now position your accel, do not move and press ENTER ..." << flush;
        getline(cin, dummy);
        if(dummy == "b") break;

        for(int j = 0; j < samples; j++) {
            sample = get_next_sample();
            accumulated += sample.acc_g;
        }

        accumulated /= samples;

        measurements.push_back(accumulated);

        cout << "added" << endl;
    }
    cout << "Computing ..." << endl;

    AccelSolver solver(measurements);

    double error = 0;
    arma::vec solution = solver.solve(&error);

    cout << "Fitness: " << error << endl;

    calibration.accel_cal = solution;

    return solution;
}

arma::vec MPU::calibrate_gyr(int samples) {
    MPU_sample sample;
    arma::vec accumulated(3);

    for(int j = 0; j < samples; j++) {
        sample = get_next_sample();
        accumulated += sample.gyr_degs;
    }

    accumulated /= -samples;

    calibration.gyro_cal = accumulated;

    return accumulated;
}


MPU_sample MPU::transform_by_calibration(MPU_sample s) {
    s.acc_g -= calibration.accel_cal.subvec(0,2);
    s.acc_g /= calibration.accel_cal.subvec(3,5);
    s.gyr_degs += calibration.gyro_cal;
    return s;
}

MPU_sample MPU::get_next_sample_cal(){
    return transform_by_calibration(get_next_sample());
}

bool MPU::load_calibration(string filename) {
    std::ifstream file(filename);
    bool ret = calibration.accel_cal.load(file);
    file.close();
    return ret;
}

bool MPU::save_calibration(string filename) {
    std::ofstream file(filename);
    bool ret = calibration.accel_cal.save(file);
    file.close();
    return ret;
}
