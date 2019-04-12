inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "DR Plugin server"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps-noship/dr_amt/drplugin_server/"
S = "${WORKDIR}/gps-noship/dr_amt/drplugin_server"
DEPENDS = "glib-2.0 qmi-framework diag slim-client slim-common slim-utils lbs-core loc-externaldr gps-utils loc-pla"
EXTRA_OECONF = "--with-glib"

