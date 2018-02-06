AM_CFLAGS = \
     $(LOCBASEUTIL_CFLAGS) \
     -I./inc/ \
     -I./inc/internal \
     $(LOCPLA_CFLAGS) \
     $(GPSUTILS_CFLAGS) \
     -D_HAS_LOC_V02_ \
	 -D__func__=__PRETTY_FUNCTION__ \
     -DON_TARGET_TEST

ACLOCAL_AMFLAGS = -I m4

libdataitems_la_SOURCES = \
    src/DataItemsFactory.cpp \
    src/DataItemBase.cpp \
    src/DataItemConcreteTypes.cpp \
    src/DataItemSerializerBase.cpp \
    src/DataItemSerializerConcreteTypes.cpp \
    src/DataItemDeSerializerBase.cpp \
    src/DataItemDeSerializerConcreteTypes.cpp \
    src/DataItemCopierBase.cpp \
    src/DataItemCopierConcreteTypes.cpp \
    src/DataItemStringifierBase.cpp \
    src/DataItemStringifierConcreteTypes.cpp

if USE_GLIB
libdataitems_la_CFLAGS = -DUSE_GLIB $(AM_CFLAGS) @GLIB_CFLAGS@
libdataitems_la_LDFLAGS = -lstdc++ -Wl,-z,defs -lpthread @GLIB_LIBS@ -shared -version-info 1:0:0
libdataitems_la_CPPFLAGS = -DUSE_GLIB $(AM_CFLAGS) $(AM_CPPFLAGS) @GLIB_CFLAGS@
else
libdataitems_la_CFLAGS = $(AM_CFLAGS)
libdataitems_la_LDFLAGS = -Wl,-z,defs -lpthread -shared -version-info 1:0:0
libdataitems_la_CPPFLAGS = $(AM_CFLAGS) $(AM_CPPFLAGS)
endif

libdataitems_la_LIBADD = -lstdc++ $(LOCBASEUTIL_LIBS) $(LOCPLA_LIBS) $(LOC_LIBS) $(GPSUTILS_LIBS)

library_include_HEADERS = \
    inc/DataItemConcreteTypeDefaultValues.h \
    inc/DataItemConcreteTypeFieldNames.h \
    inc/DataItemId.h \
    inc/DataItemsFactory.h \
    inc/IDataItem.h \
    inc/DataItemBase.h \
    inc/DataItemConcreteTypes.h \
    inc/IDataItemSerializer.h \
    inc/DataItemSerializerBase.h \
    inc/DataItemSerializerConcreteTypes.h \
    inc/IDataItemDeSerializer.h \
    inc/DataItemDeSerializerBase.h \
    inc/DataItemDeSerializerConcreteTypes.h \
    inc/IDataItemCopier.h \
    inc/DataItemCopierBase.h \
    inc/DataItemCopierConcreteTypes.h \
    inc/IDataItemStringifier.h \
    inc/DataItemStringifierBase.h \
    inc/DataItemStringifierConcreteTypes.h \
    inc/loc_ril.h

#Create and Install libraries
lib_LTLIBRARIES = libdataitems.la
library_includedir = $(pkgincludedir)
pkgconfigdir = $(libdir)/pkgconfig

pkgconfig_DATA = data-items.pc
EXTRA_DIST = $(pkgconfig_DATA)
