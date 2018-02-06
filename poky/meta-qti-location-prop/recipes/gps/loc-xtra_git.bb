inherit autotools qcommon qlicense
DESCRIPTION = "GPS Location Xtra library"
PR = "r3"
SRC_DIR = "${WORKSPACE}/gps/location-xtra/"
S = "${WORKDIR}/gps/location-xtra"
DEPENDS = "loc-pla loc-hal"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
