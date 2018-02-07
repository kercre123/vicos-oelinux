ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS := \
              -I.

bsdiff_SOURCES := bsdiff.c
bsdiff_CPPFLAGS := $(AM_CPPFLAGS)
bsdiff_LDFLAGS :=
bsdiff_LDADD := -lbz2

bspatch_SOURCES := bspatch.c
bspatch_CPPFLAGS := $(AM_CPPFLAGS)
bspatch_LDFLAGS :=
bspatch_LDADD := -lbz2

bin_PROGRAMS := bsdiff bspatch
