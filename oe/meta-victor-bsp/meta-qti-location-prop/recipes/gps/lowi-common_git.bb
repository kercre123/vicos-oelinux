inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS lowi common"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/internal/lowi/common/"
SRC_URI = "file://gps-noship/internal/lowi/common/"
S = "${WORKDIR}/gps-noship/internal/lowi/common/"
DEPENDS = "glib-2.0 loc-base-util loc-mq-client loc-pla"
EXTRA_OECONF = "--with-glib"
