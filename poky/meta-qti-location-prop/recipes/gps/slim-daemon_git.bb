inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "SLIM daemon "
PR = "r1"
SRC_DIR = "${WORKSPACE}/gps/slim/"
DEPENDS = "glib-2.0 qmi-framework diag gps-utils slim-utils slim-common slim-client canwrapper loc-pla libhardware libsensors"
EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR} \
                --with-glib \
                --enable-target=${BASEMACHINE} \
                --enable-vnw=yes"
S = "${WORKDIR}/gps/slim"
