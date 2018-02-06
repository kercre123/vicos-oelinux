inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "SLIM Common library"
PR = "r1"
SRC_DIR = "${WORKSPACE}/gps-noship/slim/"
S = "${WORKDIR}/gps-noship/slim"
DEPENDS = "glib-2.0 qmi-framework loc-hal diag slim-utils loc-pla"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"

