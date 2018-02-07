# Build qmmf test outplace algorithm library
lib_LTLIBRARIES = libqmmf_test_algo_outplace.la

libqmmf_test_algo_outplace_la_SOURCES = sample-plugins/qmmf_test_algo_outplace.cc

libqmmf_test_algo_outplace_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_test_algo_outplace_la_LDFLAGS  = -llog -lutils -lcutils

# Build qmmf test inplace algorithm library
lib_LTLIBRARIES += libqmmf_test_algo_inplace.la

libqmmf_test_algo_inplace_la_SOURCES = sample-plugins/qmmf_test_algo_inplace.cc

libqmmf_test_algo_inplace_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_test_algo_inplace_la_LDFLAGS  = -llog -lutils -lcutils

# Build qmmf test outplace algorithm with history library
lib_LTLIBRARIES += libqmmf_test_algo_outplace_history.la

libqmmf_test_algo_outplace_history_la_SOURCES = sample-plugins/qmmf_test_algo_outplace_history.cc

libqmmf_test_algo_outplace_history_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_test_algo_outplace_history_la_LDFLAGS  = -llog -lutils -lcutils

# Build qmmf test inplace algorithm with history library
lib_LTLIBRARIES += libqmmf_test_algo_inplace_history.la

libqmmf_test_algo_inplace_history_la_SOURCES = sample-plugins/qmmf_test_algo_inplace_history.cc

libqmmf_test_algo_inplace_history_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_test_algo_inplace_history_la_LDFLAGS  = -llog -lutils -lcutils

# Build qmmf test resizer library
lib_LTLIBRARIES += libqmmf_test_algo_resizer.la

libqmmf_test_algo_resizer_la_SOURCES = sample-plugins/qmmf_test_algo_resizer.cc

libqmmf_test_algo_resizer_la_CPPFLAGS = -I$(top_srcdir)/include
libqmmf_test_algo_resizer_la_LDFLAGS  = -llog -lutils -lcutils

# Build qmmf algorithm interface gtest application binary
bin_PROGRAMS = qmmf_algo_interface_gtest

qmmf_algo_interface_gtest_SOURCES = gtest/qmmf_algo_interface_gtest.cc

qmmf_algo_interface_gtest_CPPFLAGS = -I$(top_srcdir)/include
qmmf_algo_interface_gtest_LDFLAGS  = -lutils -lcutils -llog -lgtest -lgtest_main
qmmf_algo_interface_gtest_LDFLAGS  += -ldl

# Build qmmf algorithm interface gtest application binary
bin_PROGRAMS += qmmf_utils_gtest

qmmf_utils_gtest_SOURCES = gtest/qmmf_utils_gtest.cc

qmmf_utils_gtest_CPPFLAGS = -I$(top_srcdir)/include
qmmf_utils_gtest_LDFLAGS  = -lutils -lcutils -llog -lgtest -lgtest_main
qmmf_utils_gtest_LDFLAGS  += -ldl
