inherit autotools-brokensep pkgconfig qcommon qlicense qprebuilt

DESCRIPTION = "GPS Location Service library"
PR = "r1"

FILESPATH =+ "${WORKSPACE}/gps/framework:"
SRC_URI = "file://native"
SRC_DIR = "${WORKSPACE}/gps/framework/native"
S = "${WORKDIR}/native"
DEPENDS = "glib-2.0 loc-pla loc-hal gps-utils lbs-core lowi-client ulp2 loc-flp-hdr data-items loc-mq-client"
EXTRA_OECONF = "--with-glib"
