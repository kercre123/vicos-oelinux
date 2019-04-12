inherit autotools-brokensep pkgconfig qlicense qprebuilt
DESCRIPTION = "MCM Location Test Library"
PR = "r4"

FILESPATH        =+ "${WORKSPACE}:"
SRC_URI = "file://mcm-gps/loc-mcm-test-shim/"
S = "${WORKDIR}/mcm-gps/loc-mcm-test-shim/"
SRC_DIR = "${WORKSPACE}/mcm-gps/loc-mcm-test-shim/"

DEPENDS = "mcm gps-utils garden-test-interfaces loc-mcm-type-conv"

EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
