inherit autotools qlicense qcommon qprebuilt

DESCRIPTION = "GPS HAL API Test shim Library"
PR = "r3"
SRC_DIR = "${WORKSPACE}/gps/garden-app/loc-hal-test-shim/"
S = "${WORKDIR}/gps/garden-app/loc-hal-test-shim"

DEPENDS = "garden-test-interfaces loc-pla loc-hal gps-utils location-service"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
