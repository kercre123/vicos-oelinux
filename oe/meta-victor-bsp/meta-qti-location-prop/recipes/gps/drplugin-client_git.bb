inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "DR Plugin client"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps-noship/dr_amt/drplugin_client/"
S = "${WORKDIR}/gps-noship/dr_amt/drplugin_client"
DEPENDS = "glib-2.0 gps-utils lbs-core loc-pla"
EXTRA_OECONF = "--with-glib"

