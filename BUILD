#!/bin/bash

cd src/hidapi/
./bootstrap
./configure
make
make DESTDIR=$PWD/install install
cd -

cd src/imu_host/build/
cmake ..
make
cd -

cd catkin_ws
catkin_make
