inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "SLIM cleint "
PR = "r1"
SRC_DIR = "${WORKSPACE}/gps/slim/client/"
DEPENDS = "glib-2.0 qmi-framework diag gps-utils slim-utils slim-common loc-pla"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
S = "${WORKDIR}/gps/slim/client"
