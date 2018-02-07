# Build player test application binary
bin_PROGRAMS = player_test

player_test_SOURCES  = qmmf_player_test.cc
player_test_CPPFLAGS = $(CFLAGS)
player_test_CPPFLAGS += -I$(top_srcdir)/include
player_test_CPPFLAGS += -I$(top_srcdir)/
player_test_LDFLAGS  = -lutils -lcutils -llog -lpthread
player_test_LDADD    = ../../src/client/libqmmf_player_client.la
player_test_LDADD    += ../demuxer/libdemuxer.la

# Build player test application binary
bin_PROGRAMS += player_parser_test

player_parser_test_SOURCES  = qmmf_player_parser_test.cc
player_parser_test_SOURCES  += qmmf_player_parser.cc
player_parser_test_CPPFLAGS = -I$(top_srcdir)/include
player_parser_test_CPPFLAGS += -I$(top_srcdir)/
player_parser_test_LDFLAGS  = -lutils -lcutils -llog -lpthread
player_parser_test_LDADD    = ../../src/client/libqmmf_player_client.la
