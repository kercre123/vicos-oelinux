# Build libqmmf_postproc_algo
lib_LTLIBRARIES = libqmmf_postproc_algo.la

libqmmf_postproc_algo_la_SOURCES = qmmf_postproc_algo.cc

libqmmf_postproc_algo_la_CPPFLAGS  = -I$(top_srcdir)/include
libqmmf_postproc_algo_la_CPPFLAGS += -I$(top_srcdir)/
libqmmf_postproc_algo_la_CPPFLAGS += -I$(LIBGRALLOC)

libqmmf_postproc_algo_la_LDFLAGS  = -lutils -lpthread -ldl -llog -lbinder -lcutils
