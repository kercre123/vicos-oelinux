inherit autotools-brokensep pkgconfig qcommon qlicense qprebuilt

DESCRIPTION = "Location HAL Tests"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps/location_hal_test/"
S = "${WORKDIR}/gps/location_hal_test"

DEPENDS = "glib-2.0 loc-pla loc-hal location-geofence location-flp lbs-core lbs-core-hdr"

CXXINC  = "-I${STAGING_INCDIR}/c++"
CXXINC += "-I${STAGING_INCDIR}/c++/${TARGET_SYS}"
CXXFLAGS ="${CXXINC}"

EXTRA_OECONF = "--with-glib"


