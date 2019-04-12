inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Lib DRPlugin"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps-noship/dr_amt/drplugin/"
S = "${WORKDIR}/gps-noship/dr_amt/drplugin"
DEPENDS = "glib-2.0 qmi-framework loc-hal gps-utils lbs-core gnsspps drplugin-client loc-pla"
EXTRA_OECONF = "--with-glib"

