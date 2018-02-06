inherit autotools-brokensep pkgconfig qcommon qlicense qprebuilt

DESCRIPTION = "GPS LBS Core"
PR = "r1"

FILESPATH =+ "${WORKSPACE}/gps:"
SRC_URI = "file://ulp2"
SRC_DIR = "${WORKSPACE}/gps/ulp2"
DEPENDS = "glib-2.0 qmi-framework loc-pla loc-hal gps-utils izat-core lbs-core drplugin event-observer loc-flp-hdr gnsspps drplugin-client"
EXTRA_OECONF ="--with-glib"

S = "${WORKDIR}/ulp2"
