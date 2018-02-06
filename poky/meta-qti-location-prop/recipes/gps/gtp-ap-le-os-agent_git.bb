inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS xtwifi LE os agent"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/xtwifi/gtp-ap-le-os-agent/"
S = "${WORKDIR}/gps-noship/xtwifi/gtp-ap-le-os-agent"
DEPENDS = "glib-2.0 loc-base-util loc-mq-client"
EXTRA_OECONF = "--with-glib"
