inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS lowi server"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/internal/lowi/lowi_server/"
S = "${WORKDIR}/gps-noship/internal/lowi/lowi_server"
DEPENDS = "glib-2.0 loc-hal loc-pla loc-base-util loc-launcher lowi-client libnl"
EXTRA_OECONF = "--with-glib"
