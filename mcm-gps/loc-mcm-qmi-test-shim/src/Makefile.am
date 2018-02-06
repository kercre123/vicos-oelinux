AM_CFLAGS = \
            -I. \
            $(GTI_CFLAGS) \
            $(MCMCORE_CFLAGS) \
            $(QMIF_CFLAGS) \
            $(LOCMCMTYPECONV_CFLAGS) \
            $(GPSUTILS_CFLAGS) \
            -DHAVE_LIBLOG

requiredlibs = $(MCMCORE_LIBS) $(QMIF_LIBS) $(LOCMCMTYPECONV_LIBS) $(GPSUTILS_LIBS)

h_sources = \
    location_callbacks_qmi_loc.h \
    location_test_interface_qmi_loc.h

c_sources = \
    location_test_interface_qmi_loc.cpp

library_includedir = $(pkgincludedir)
library_include_HEADERS = $(h_sources)

libloc_mcm_qmi_test_shim_la_SOURCES = $(c_sources) $(h_sources)

if USE_GLIB
libloc_mcm_qmi_test_shim_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libloc_mcm_qmi_test_shim_la_LDFLAGS = -lstdc++ -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libloc_mcm_qmi_test_shim_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libloc_mcm_qmi_test_shim_la_CFLAGS = $(AM_CFLAGS)
libloc_mcm_qmi_test_shim_la_LDFLAGS = -shared -version-info 1:0:0
libloc_mcm_qmi_test_shim_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libloc_mcm_qmi_test_shim_la_LIBADD = $(requiredlibs)

lib_LTLIBRARIES = libloc_mcm_qmi_test_shim.la
