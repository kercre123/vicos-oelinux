#Portions of this file are copied from GStreamer template which GStreamer provides under the MIT license.

plugin_LTLIBRARIES = libgstqahw.la

libgstqahw_la_SOURCES = \
	gstqahwplugin.c \
	gstqahwsrc.c \
	gstqahwsink.c

QAHW_EFFECT_SOURCES = \
	gstqahweffect.c \
	gstqahwequalizer.c \
	gstqahwaudiosphere.c \
	gstqahwbassboost.c \
	gstqahwpresetreverb.c \
	gstqahwvirtualizer.c

if USE_QAHW_EFFECT
libgstqahw_la_SOURCES += $(QAHW_EFFECT_SOURCES)
else
EXTRA_DIST = $(QAHW_EFFECT_SOURCES)
endif

libgstqahw_la_CFLAGS = \
        -I$(PKG_CONFIG_SYSROOT_DIR)/usr/include/mm-audio/qahw_api/inc \
        $(QAHW_CFLAGS) \
	$(GST_PLUGINS_BASE_CFLAGS) \
	$(GST_BASE_CFLAGS) \
	$(GST_CFLAGS) \
	$(ALSA_CFLAGS)
libgstqahw_la_LIBADD = \
        $(QAHW_LIBS) \
	-lgstaudio-@GST_API_VERSION@ \
	$(GST_BASE_LIBS) \
	$(ALSA_LIBS)

libgstqahw_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS)
libgstqahw_la_LIBTOOLFLAGS = $(GST_PLUGIN_LIBTOOLFLAGS)

noinst_HEADERS = \
	gstqahwsrc.h \
	gstqahwsink.h \
	gstqahweffect.h \
	gstqahwequalizer.h \
	gstqahwaudiosphere.h \
	gstqahwbassboost.h \
	gstqahwpresetreverb.h \
	gstqahwvirtualizer.h \
	gstqahw-common.h
