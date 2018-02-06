AM_CFLAGS = -Wall -g \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        $(SECUREMSMNOSHIP_CFLAGS)

ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -D__packed__= \
    -DFEATURE_LE_DIAG \
    -DOE \
    -I$(WORKSPACE)/security/securemsm/keymaster \
    $(SECUREMSMNOSHIP_CFLAGS)

bin_PROGRAMS = keymaster_sample_client

keymaster_sample_client_CC = @CC@
pkgconfigdir = $(libdir)/pkgconfig
keymaster_sample_client_SOURCES = keymaster_sample_client.c
keymaster_sample_client_CPPFLAGS = $(AM_CPPFLAGS)
keymaster_sample_client_CFLAGS = $(AM_CFLAGS)
keymaster_sample_client_LDFLAGS = -lpthread -ldl $(SECUREMSMNOSHIP_LIBS) -lm -lKeyMaster
keymaster_sample_client_LDADD = $(SECUREMSMNOSHIP_LIBS)
