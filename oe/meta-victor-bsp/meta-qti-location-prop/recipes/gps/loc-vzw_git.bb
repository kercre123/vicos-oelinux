inherit autotools qlicense qcommon qprebuilt

DESCRIPTION = "GPS Location vzw"
PR = "r1"
SRC_DIR = "${WORKSPACE}/gps/vzwGpsLocationProvider/loc_ext/"
S = "${WORKDIR}/gps/vzwGpsLocationProvider/loc_ext"

DEPENDS = "qmi-framework loc-pla gps-utils loc-hal loc-flp-hdr"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib"
