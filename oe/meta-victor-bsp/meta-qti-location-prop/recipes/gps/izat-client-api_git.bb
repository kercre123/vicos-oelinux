inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Izat client api libary"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps/framework/native/lcp/izat_api/"
S = "${WORKDIR}/gps/framework/native/lcp/izat_api"
DEPENDS = "glib-2.0 gps-utils loc-base-util loc-mq-client izat-api-hdr location-service loc-pla"
EXTRA_OECONF ="--with-glib \
               --with-izatapi-includes=${STAGING_INCDIR}/izat-api-hdr"

