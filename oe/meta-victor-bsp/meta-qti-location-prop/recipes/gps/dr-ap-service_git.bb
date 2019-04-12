inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "DR AP service"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps-noship/dr_amt/DR_AP_service/"
S = "${WORKDIR}/gps-noship/dr_amt/DR_AP_service"
DEPENDS = "glib-2.0 drplugin-server gps-utils loc-pla loc-externaldrcore"
EXTRA_OECONF = "--with-glib"

