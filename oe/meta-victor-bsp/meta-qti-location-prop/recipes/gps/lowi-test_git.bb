inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS lowi test"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/internal/lowi/test/"
S = "${WORKDIR}/gps-noship/internal/lowi/test"
DEPENDS = "glib-2.0 loc-base-util loc-launcher lowi-client loc-pla libxml2"
EXTRA_OECONF = "--with-glib"
