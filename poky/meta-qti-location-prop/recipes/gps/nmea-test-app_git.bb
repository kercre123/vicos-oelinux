inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "NMEA test application "
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps/test/nmea_test_app/"
S = "${WORKDIR}/gps/test/nmea_test_app/"
DEPENDS = "glib-2.0 loc-hal"
EXTRA_OECONF ="--with-glib"

