source_files = qmmf_display_service.cc
source_files += qmmf_display_impl.cc
source_files += qmmf_remote_cb.cc
source_files += qmmf_display_sdm_buffer_allocator.cc
source_files += qmmf_display_sdm_debugger.cc
source_files += qmmf_display_sdm_buffer_sync_handler.cc


# Build qmmf display service library
# libqmmf_display_service.so
lib_LTLIBRARIES = libqmmf_display_service.la

libqmmf_display_service_la_SOURCES = $(source_files)

libqmmf_display_service_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_display_service_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_display_service_la_CPPFLAGS += -I$(LIBGRALLOC)
libqmmf_display_service_la_CPPFLAGS += -I$(MMCORE_CFLAGS)
libqmmf_display_service_la_CPPFLAGS += -I${WORKSPACE}/display/display-hal/sdm/include/
libqmmf_display_service_la_LDFLAGS  = -llog -lbinder -lutils -lcutils
libqmmf_display_service_la_LDFLAGS  += -lsdmcore -lsdmutils -lmemalloc

