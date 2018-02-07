AM_CFLAGS = -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -g -O0 \
        -fno-inline \
        -fno-short-enums \
        -fpic \
        -I${top_srcdir}/include

AM_CPPFLAGS = $(AM_CFLAGS)

ACLOCAL_AMFLAGS = -I m4

### Build libsparse library

libsparse_h_sources = include/sparse/sparse.h
libsparse_c_sources := append2simg.c \
        backed_block.c \
        output_file.c \
        sparse.c \
        sparse_crc32.c \
        sparse_err.c \
        sparse_read.c

libsparse_la_SOURCES = $(libsparse_c_sources) $(libsparse_h_sources)
libsparse_la_LDFLAGS = $(Z_LIB)

### Export library
lib_LTLIBRARIES = libsparse.la

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libsparse.pc
EXTRA_DIST = $(pkgconfig_DATA)

### Export headers
library_includedir = $(includedir)/sparse
library_include_HEADERS = $(libsparse_h_sources)
