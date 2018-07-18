/*******************************************************
 Windows HID simplification

 Alan Ott
 Signal 11 Software

 8/22/2009

 Copyright 2009
 
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/

#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "hidapi.h"
#define SWAP_NIBBLES(x) (((uint16_t)x << 8) | ((uint16_t)x >> 8))
#define MAX_STR 256
// Headers needed for sleeping.
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

int main(int argc, char* argv[])
{
    int res;
    unsigned char buf[256];
    wchar_t wstr[MAX_STR];
    hid_device *handle;
    int i;

    struct hid_device_info *devs, *cur_dev;

    if (hid_init())
        return -1;

    // Open the device using the VID, PID,
    handle = hid_open(0x0483, 0x5710, NULL);
    if (!handle) {
        printf("unable to open device\n");
        return 1;
    }

    // Read the Product String
    wstr[0] = 0x0000;
    res = hid_get_product_string(handle, wstr, MAX_STR);
    if (res < 0)
        printf("Unable to read product string\n");
    printf("Product String: %ls\n", wstr);

    // Measure response time???
    for(i = 0 ; i < 100 ; i ++) {
        res = hid_get_feature_report(handle, buf, 16);
        if (res < 0) {
            printf("Unable to send a feature report.\n");
        }
        res = hid_get_feature_report(handle, buf + 5, 16);
        if (res < 0) {
            printf("Unable to get a feature report.\n");
        }
        uint32_t t1 = buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];
        uint32_t t2 = buf[6] << 24 | buf[7] << 16 | buf[8] << 8 | buf[9];
        double t =  (t2 - t1) / 120000.0;
        printf("%f\t%u %u\n", t, t1, t2);
        usleep(5000);
    }


    //memset(zbuf, 0, 32);
    //zbuf[0] = SWAP_NIBBLES(-10000);
    //res = hid_send_feature_report(handle, (unsigned char*)zbuf, 12);
    //if (res < 0) {
    //    printf("Unable to send a feature report.\n");
    //}

    // Set the hid_read() function to be non-blocking.
    hid_set_nonblocking(handle, 1);

    memset(buf,0,sizeof(buf));

    // Read a Feature Report from the device
    res = hid_get_feature_report(handle, buf, sizeof(buf));
    if (res < 0) {
        printf("Unable to get a feature report.\n");
        printf("%ls", hid_error(handle));
    }
    else {
        // Print out the returned buffer.
        printf("Feature Report\n   ");
        for (i = 0; i < res; i++)
            printf("%02hhx ", buf[i]);
        printf("\n");
    }

    memset(buf,0,sizeof(buf));

    res = 0;
    uint32_t maxtime = 0, err = 0;
    while (1) {
        res = hid_read(handle, buf, sizeof(buf));
        if (res == 0) {
            usleep(500);
            continue;
        }
        if (res < 0) {
            printf("Unable to read()\n");
            break;
        }

        uint32_t time1 = buf[12] << 24 | buf[13] << 16 | buf[14] << 8 | buf[15];
        uint32_t time2 = buf[28] << 24 | buf[29] << 16 | buf[30] << 8 | buf[31];
        int16_t accx = buf[0] << 8 | buf[1];
        double accx_f = accx;
        accx_f /= 16384.0;

        if(time1 != 0 && maxtime !=0) {
            int delta = (time1 - maxtime) > 180000 ? (time1 - maxtime)/120000 : 0;
            err += delta;
            if(delta) {
                printf("err %u %u %u\n", time1, delta, time1 - maxtime);
    //            fprintf(stderr, "err %u %u\n", time1, delta);
            }
        }
        if(time1 != 0) maxtime = time1 > time2 ? time1 : time2;
//#define PRINT_DETAILS
#ifdef PRINT_DETAILS
        printf("%u\t%u\t%f\t%u\t   ", time1, time2, accx_f, err);
        for (i = 0; i < res; i++) {
            //printf("%06d ", (int16_t)SWAP_NIBBLES(((uint16_t*) buf)[i]));
            printf("%02hhx ", buf[i]);

        }
        printf("\n");
#endif

    }

    printf("Data read:\n   ");
    for (i = 0; i < res; i++)
        printf("%02hhx ", buf[i]);
    printf("\n");

    hid_close(handle);

    /* Free static HIDAPI objects. */
    hid_exit();

#ifdef WIN32
    system("pause");
#endif

    return 0;
}
