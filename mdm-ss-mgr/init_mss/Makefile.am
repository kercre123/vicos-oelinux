M_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs

ACLOCAL_AMFLAGS = -I m4

bin_PROGRAMS = init_mss
pkgconfigdir = $(libdir)/pkgconfig
init_mss_SOURCES = init_mss.c
if SLEEP_INDEFINITE
init_mss_CFLAGS  = $(CFLAGS) -DSLEEP_INDEFINITE
endif
