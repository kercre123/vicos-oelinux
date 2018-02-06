inherit qcommon qlicense qprebuilt

DESCRIPTION = "Qualcomm Data Configdb Module"
DEPENDS = "common dsutils diag xmllib glib-2.0"
PR = "r5"

EXTRA_OECONF = "--with-lib-path=${STAGING_LIBDIR} \
                --with-common-includes=${STAGING_INCDIR} \
                --with-glib \
                --with-qxdm"

S       = "${WORKDIR}/configdb"
SRC_DIR = "${WORKSPACE}/data/configdb"
