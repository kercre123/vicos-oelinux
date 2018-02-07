# Build qmmf-server binary
bin_PROGRAMS = qmmf-server

qmmf_server_SOURCES  = qmmf_server_main.cc

qmmf_server_CPPFLAGS = -I$(top_srcdir)/include
qmmf_server_CPPFLAGS += -I$(top_srcdir)/

qmmf_server_CPPFLAGS += -I$(LIBGRALLOC)
qmmf_server_CPPFLAGS += -I$(MMCORE_CFLAGS)
qmmf_server_CPPFLAGS += -I${WORKSPACE}/display/display-hal/sdm/include/

qmmf_server_LDFLAGS  = -lutils -lcutils -llog -lbinder
qmmf_server_LDADD    = ../common/audio/src/service/libqmmf_audio_service.la
qmmf_server_LDADD    += ../recorder/src/service/libqmmf_recorder_service.la
qmmf_server_LDADD    += ../display/src/service/libqmmf_display_service.la
qmmf_server_LDADD    += ../player/src/service/libqmmf_player_service.la
qmmf_server_LDADD    += ../system/src/service/libqmmf_system_service.la
