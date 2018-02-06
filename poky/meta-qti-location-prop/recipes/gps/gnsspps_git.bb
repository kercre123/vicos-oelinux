inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "gnsspps"
PR = "r1"

SRC_DIR = "${WORKSPACE}/qcom-opensource/location/gnsspps/"
S = "${WORKDIR}/qcom-opensource/location/gnsspps"
DEPENDS = "glib-2.0 loc-pla gps-utils"
EXTRA_OECONF = "--with-glib"

