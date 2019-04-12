inherit qcommon qlicense qprebuilt

DESCRIPTION = "Qualcomm Data DSutils Module"
PR = "r7"

DEPENDS = "common diag glib-2.0 virtual/kernel libcutils"

EXTRA_OECONF = "--with-lib-path=${STAGING_LIBDIR} \
                --with-common-includes=${STAGING_INCDIR} \
                --with-glib \
                --with-qxdm \
                --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include \
                --enable-target=${BASEMACHINE}"

CFLAGS += "-I${STAGING_INCDIR}/cutils"

S       = "${WORKDIR}/dsutils"
SRC_DIR = "${WORKSPACE}/data/dsutils"
