inherit autotools-brokensep pkgconfig qlicense qprebuilt
DESCRIPTION = "MCM Location Server"
PR = "r5"

FILESPATH        =+ "${WORKSPACE}:"
SRC_URI = "file://mcm-gps/mcmlocserver/"
S = "${WORKDIR}/mcm-gps/mcmlocserver/"
SRC_DIR = "${WORKSPACE}/mcm-gps/mcmlocserver/"

DEPENDS = "glib-2.0 qmi-framework mcm mcm-core loc-mcm-type-conv loc-pla loc-hal gps-utils location-service"

EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
