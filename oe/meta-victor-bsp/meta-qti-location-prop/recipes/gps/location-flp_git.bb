inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "GPS FLP"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://gps-noship/flp/"
SRC_DIR = "${WORKSPACE}/gps-noship/flp/"
S = "${WORKDIR}/gps-noship/flp"
DEPENDS = "glib-2.0 loc-pla gps-utils loc-hal lbs-core location-geofence izat-core"
EXTRA_OECONF = "--with-glib"
