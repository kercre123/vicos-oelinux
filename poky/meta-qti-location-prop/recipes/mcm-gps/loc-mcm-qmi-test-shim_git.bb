inherit autotools-brokensep pkgconfig qlicense qprebuilt
DESCRIPTION = "MCM Location Type Conversion Library"
PR = "r4"

FILESPATH        =+ "${WORKSPACE}:"
SRC_URI = "file://mcm-gps/loc-mcm-qmi-test-shim/"
S = "${WORKDIR}/mcm-gps/loc-mcm-qmi-test-shim/"
SRC_DIR = "${WORKSPACE}/mcm-gps/loc-mcm-qmi-test-shim/"

DEPENDS = "qmi-framework mcm-core gps-utils garden-test-interfaces loc-mcm-type-conv"

EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
