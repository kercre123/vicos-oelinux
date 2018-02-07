# Build player test application binary
bin_PROGRAMS = player_gtest

player_gtest_SOURCES  = qmmf_player_gtest.cc
player_gtest_SOURCES  += qmmf_player_parser.cc
player_gtest_CPPFLAGS = -I$(top_srcdir)/include
player_gtest_CPPFLAGS += -I$(top_srcdir)/
player_gtest_LDFLAGS  = -lutils -lcutils -llog -lpthread -lgtest -lgtest_main
player_gtest_LDADD    = ../../src/client/libqmmf_player_client.la

