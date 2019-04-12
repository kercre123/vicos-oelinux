inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Event Observer"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps/utils/eventObserver/"
S = "${WORKDIR}/gps/utils/eventObserver"
DEPENDS = "glib-2.0 gps-utils loc-pla"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"

