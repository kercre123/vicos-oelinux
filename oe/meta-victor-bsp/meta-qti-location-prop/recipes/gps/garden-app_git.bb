inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "GPS Garden test shippable app for LOC API HAL"
PR = "r5"
SRC_DIR = "${WORKSPACE}/gps/garden-app/"
S = "${WORKDIR}/gps/garden-app"
DEPENDS = "qmi qmi-framework loc-pla loc-vzw loc-hal loc-hal-test-shim garden-test-interfaces \
location-service loc-glue ulp2 event-observer izat-core location-flp gps-utils"

EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"

