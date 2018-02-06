inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS xtwifi tile"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/xtwifi/inet_agent/src/"
S = "${WORKDIR}/gps-noship/xtwifi/inet_agent/src"
DEPENDS = "glib-2.0 loc-base-util loc-mq-client curl loc-pla"
EXTRA_OECONF = "--with-glib"
