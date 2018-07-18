TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

LIBS += /home/vladimir/src/hidapi/linux/.libs/libhidapi-hidraw.so
INCLUDEPATH += /home/vladimir/src/hidapi/hidapi/

LIBS += -larmadillo

SOURCES += \
        main.cpp \
    hid.cpp \
    hid_mpu.cpp \
    quaternion.cpp

HEADERS += \
    hid.h \
    gsl-lite.hpp \
    hid_mpu.h \
    accel_solver.h \
    quaternion.h
