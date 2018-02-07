AM_CFLAGS := -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -g -O0 \
        -fno-inline \
        -fno-short-enums \
        -fpic

AM_LEXFLAGS := -x c

AM_YFLAGS := -v -d

AM_CPPFLAGS := \
              -I.

AM_LDFLAGS := -shared -version-info 1:0:0
c_sources := parser.y \
            lexer.l \
            expr.c

library_includedir := $(pkgincludedir)

lib_LTLIBRARIES := libedify.la
if USE_GLIB
libedify_la_SOURCES := $(c_sources) $(h_sources)
libedify_la_CFLAGS := $(AM_CFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libedify_la_CPPFLAGS := $(AM_CPPFLAGS) -DUSE_GLIB @GLIB_CFLAGS@
libedify_la_LIBADD := -lm -lpthread @GLIB_LIBS@
libedify_la_LDFLAGS := -all-static
else
libedify_la_SOURCES := $(c_sources) $(h_sources)
libedify_la_CFLAGS := $(AM_CFLAGS)
libedify_la_CPPFLAGS := $(AM_CPPFLAGS)
libedify_la_LIBADD := -lm -lpthread
libedify_la_LDFLAGS := -all-static
endif
