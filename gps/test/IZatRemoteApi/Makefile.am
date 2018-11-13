AM_CFLAGS = \
    $(LOCPLA_CFLAGS) \
    $(GPSUTILS_CFLAGS)

required_libs = \
    $(LOCPLA_LIBS) \
    $(IZATCLIENTAPI_LIBS) \
    $(GPSUTILS_LIBS)

izat_remote_api_test_c_sources = \
     IZatRemoteApiTest.cpp

izat_remote_api_test_SOURCES = $(izat_remote_api_test_c_sources)

if USE_GLIB
izat_remote_api_test_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
izat_remote_api_test_LDFLAGS = -lstdc++ -g -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
izat_remote_api_test_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
izat_remote_api_test_CFLAGS = $(AM_CFLAGS)
izat_remote_api_test_LDFLAGS = -lpthread -Wl,-z,defs -shared -version-info 1:0:0
izat_remote_api_test_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

izat_remote_api_test_LDADD = -lcutils -ldl $(required_libs)

bin_PROGRAMS = izat_remote_api_test
