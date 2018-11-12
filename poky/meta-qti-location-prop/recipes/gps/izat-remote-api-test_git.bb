inherit qcommon qlicense qprebuilt

DESCRIPTION = "Izat remote api test application "
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps/test/IZatRemoteApi/"
S = "${WORKDIR}/gps/test/IZatRemoteApi"
DEPENDS = "glib-2.0 gps-utils izat-client-api izat-api-hdr"
EXTRA_OECONF ="--with-glib \
               --with-izatapi-includes=${STAGING_INCDIR}/izat-api-hdr"

