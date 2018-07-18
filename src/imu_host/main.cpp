#include <iostream>
#include <stdio.h>
#include "hid_mpu.h"
#include "quaternion.h"
using namespace std;
#define ACCTEST

int main()
{
    try{
        MPU hid;
        hid.open(0x0483, 0x5710);

        //cout << hid.calibrate_acc() << endl;
        //hid.save_calibration("calib.bin");

        hid.load_calibration("../../calib.bin");

        cout << hid.calibrate_gyr() << endl;

        string d;
        getline(cin, d);

#ifdef QUATTEST
        uint64_t last_time = 0;
        int cnt = 0;
        Quaternion rot(0, {1,0,0});
        while(1) {
            auto sample = hid.get_next_sample_cal();
            double dt = (sample.time_us - last_time) / 1000000.0;
            if(last_time) {
                Quaternion dq (arma::norm(sample.gyr_degs) * dt / 180.0 * M_PI, normalise(sample.gyr_degs));
                rot = rot * dq;
                rot = rot * (1.0 / rot.length());
                if(cnt % 100 == 0) cout << rot.rotate({1,0,0}) << "  " << (sample.time_us - last_time) << endl;
            }
            last_time = sample.time_us;
            cnt++;
        }
#elif defined GYRTEST
        uint64_t last_time = 0;
        double degz = 0;
        int cnt = 0;
        while(1) {
            auto sample = hid.get_next_sample_cal();
            double dt = (sample.time_us - last_time) / 1000000.0;
            if(last_time) degz += sample.gyr_degs[2] * dt;
            if(cnt%100 == 0) cout << degz << "  " << (sample.time_us - last_time) << endl;
            last_time = sample.time_us;
            cnt++;
        }
#elif defined ACCTEST
        while(1) {
            arma::vec v(3), g(3);
            for(int i = 0; i < 300; i++) {
                auto s = hid.get_next_sample_cal();
                v += s.acc_g;
                g += s.gyr_degs;
            }
            v /= 300;
            g /= 300;
            cout << arma::norm(v) << "                  " << v[0] << "\t" << v[1] << "\t" << v[2] << "        "
                 << g[0] << "\t" << g[1] << "\t" << g[2] << endl;
        }
#endif

        hid.close();
    } catch(HID_exception e) {
        cout << e.get_desc() << endl;
    }
    return 0;
}
