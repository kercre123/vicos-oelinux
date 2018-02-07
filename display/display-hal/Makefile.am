# Makefile.am - Automake script for sdm

ACLOCAL_AMFLAGS = -I m4

SUBDIRS = libqservice libqdutils libgralloc sdm/libs/utils
if ENABLE_SDMHALDRM
SUBDIRS += libdrmutils
SUBDIRS += gpu_tonemapper
endif
SUBDIRS += sdm/libs/core


