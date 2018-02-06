AM_CFLAGS = -Wall -g \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        $(SECUREMSMNOSHIP_CFLAGS)

ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS = -D__packed__= \
    -DFEATURE_LE_DIAG \
    -DOE \
    $(SECUREMSMNOSHIP_CFLAGS)

#init_ddir = ${sysconfdir}/init.d
#init_d_SCRIPTS = start_qseecomd

if TARGET_SUPPORT_RPMB
    qseecomd_CPPFLAGS = $(AM_CPPFLAGS) -DSUPPORT_RPMB
    qseecomd_CFLAGS = $(AM_CFLAGS) -DSUPPORT_RPMB
else
    qseecomd_CPPFLAGS = $(AM_CPPFLAGS)
    qseecomd_CFLAGS = $(AM_CFLAGS)
endif

bin_PROGRAMS = qseecomd
qseecomd_CC = @CC@
pkgconfigdir = $(libdir)/pkgconfig
qseecomd_SOURCES = qseecomd.c
qseecomd_LDFLAGS = -lpthread -ldl $(SECUREMSMNOSHIP_LIBS)
qseecomd_LDADD = $(SECUREMSMNOSHIP_LIBS)
