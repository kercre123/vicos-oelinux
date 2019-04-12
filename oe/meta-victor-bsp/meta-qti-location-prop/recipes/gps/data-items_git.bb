inherit autotools-brokensep pkgconfig qcommon qlicense qprebuilt

DESCRIPTION = "GPS Data Items"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps/framework/native/lcp/data-items/"
S = "${WORKDIR}/gps/framework/native/lcp/data-items"
DEPENDS = "common glib-2.0 loc-pla loc-base-util gps-utils"
EXTRA_OECONF = "--with-libhardware-includes=${STAGING_INCDIR} \
                --with-glib"
