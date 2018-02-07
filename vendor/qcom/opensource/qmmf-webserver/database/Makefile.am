# Build libqmmf_database
lib_LTLIBRARIES = libqmmf_database.la

libqmmf_database_la_SOURCES := src/qmmf_database.cpp

libqmmf_database_la_CPPFLAGS := -I$(SQLITE_CFLAGS)
libqmmf_database_la_CPPFLAGS += -I$(top_srcdir)/database/inc
libqmmf_database_la_CPPFLAGS += -std=c++14 -std=gnu++0x -std=c++0x -std=gnu++14 -Wall
libqmmf_database_la_LDFLAGS  := -lutils -lpthread -ldl -llog
