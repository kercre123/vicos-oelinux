inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "LOC NV Manager"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://gps-noship/xtwifi/client_core/"
SRC_DIR = "${WORKSPACE}/gps-noship/xtwifi/client_core/"
S = "${WORKDIR}/gps-noship/xtwifi/client_core"
DEPENDS = "glib-2.0 loc-base-util loc-hal loc-pla"
EXTRA_OECONF = "--with-glib"

