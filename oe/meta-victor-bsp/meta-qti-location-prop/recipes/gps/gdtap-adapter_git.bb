inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS xtwifi gdtap"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/gdtap/"
SRC_URI = "file://gps-noship/gdtap/"
S = "${WORKDIR}/gps-noship/gdtap"
DEPENDS = "glib-2.0 loc-base-util loc-launcher loc-pla izat-core lbs-core"
EXTRA_OECONF = "--with-glib"
