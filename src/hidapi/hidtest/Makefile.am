AM_CPPFLAGS = -I$(top_srcdir)/hidapi/

## Linux
if OS_LINUX
noinst_PROGRAMS = hidtest-libusb hidtest-hidraw hid_mpu hid_mpu_lu

hidtest_hidraw_SOURCES = hidtest.cpp
hidtest_hidraw_LDADD = $(top_builddir)/linux/libhidapi-hidraw.la

hidtest_libusb_SOURCES = hidtest.cpp
hidtest_libusb_LDADD = $(top_builddir)/libusb/libhidapi-libusb.la

hid_mpu_SOURCES = hid_mpu.cpp
hid_mpu_LDADD = $(top_builddir)/linux/libhidapi-hidraw.la

hid_mpu_lu_SOURCES = hid_mpu.cpp
hid_mpu_lu_LDADD = $(top_builddir)/libusb/libhidapi-libusb.la
else

# Other OS's
noinst_PROGRAMS = hidtest

hidtest_SOURCES = hidtest.cpp
hidtest_LDADD = $(top_builddir)/$(backend)/libhidapi.la

endif
