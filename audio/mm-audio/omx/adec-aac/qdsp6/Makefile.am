pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = ../../omx.pc
EXTRA_DIST = $(pkgconfig_DATA)

AM_CFLAGS = -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -g -O0 \
        -fno-inline \
        -fno-short-enums \
        -fpic \
        -I ./inc/ \
        -I ./src/common/ \
        -I ../../alsa-utils/qdsp6/inc \
         $(MM_CORE_CFLAGS) \
         $(AUDIOALSA_CFLAGS)

AM_CPPFLAGS = -I ./inc/ \
              -I $(WORKSPACE)/system/core/include/ \
              $(MM_CORE_CFLAGS)

c_sources = src/adec_svr.c \
            src/omx_aac_adec.cpp

lib_LTLIBRARIES = libOmxAacDec.la
libOmxAacDec_la_CC = @CC@
libOmxAacDec_la_SOURCES = $(c_sources)
libOmxAacDec_la_CFLAGS = $(AM_CFLAGS) -DUSE_GLIB @GLIB_CFLAGS@ -DQC_MODIFIED -D_ENABLE_QC_MSG_LOG_ -DVERBOSE -D_DEBUG -DAUDIOV2 -Wconversion
libOmxAacDec_la_CPPFLAGS = $(AM_CPPFLAGS) -Dstrlcpy=g_strlcpy -DUSE_GLIB @GLIB_CFLAGS@ -DQC_MODIFIED -D_ENABLE_QC_MSG_LOG_ -DVERBOSE -D_DEBUG -DAUDIOV2 -Wconversion -include glib.h -DLOGV=ALOGV -include log/log.h
libOmxAacDec_la_LDFLAGS = $(GLIB_LIBS) -llog -shared -avoid-version
