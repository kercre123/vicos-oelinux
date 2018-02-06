# Makefile.am - Automake script for ss-restart
#
ACLOCAL_AMFLAGS = -I m4

#64 bits machine, it need to support compile 32bit camera and 32bit mmi_agent
if AARCH64_32COMPILER
SUBDIRS = libmmi mmi module/camera
else
if ARM64
SUBDIRS = libmmi mmi qti-system-daemon module/cpu module/battery module/memory module/storage module/wifi module/sysinfo module/sensor module/usb module/audio module/bluetooth
else
#For 32 bits machine, only compile it directly.
SUBDIRS = libmmi mmi qti-system-daemon module/cpu module/battery module/memory module/storage module/wifi module/sysinfo module/sensor module/usb module/camera module/audio module/bluetooth
endif
endif
