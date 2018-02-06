inherit autotools-brokensep pkgconfig qlicense qprebuilt
DESCRIPTION = "MCM Location Type Conversion Library"
PR = "r4"

FILESPATH        =+ "${WORKSPACE}:"
SRC_URI = "file://mcm-gps/loc-mcm-type-conv/"
S = "${WORKDIR}/mcm-gps/loc-mcm-type-conv/"
SRC_DIR = "${WORKSPACE}/mcm-gps/loc-mcm-type-conv/"

DEPENDS = "gps-utils mcm-core"

EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
