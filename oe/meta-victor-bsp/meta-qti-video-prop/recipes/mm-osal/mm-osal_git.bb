inherit qcommon autotools  deploy qlicense qprebuilt pkgconfig sdllvm

DESCRIPTION = "mm-osal"

SRC_DIR = "${WORKSPACE}/video/lib/mm-osal/"
S = "${WORKDIR}/video/lib/mm-osal/"

PR = "r4"

EXTRA_OECONF_append =" --with-utils-headers=${STAGING_INCDIR}/utils/"
EXTRA_OECONF_append =" --with-cutils-headers=${STAGING_INCDIR}/cutils/"
EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/"
EXTRA_OECONF_append =" --with-glib"

PACKAGES = "${PN}-dbg ${PN} ${PN}-dev"

DEPENDS += "glib-2.0"
DEPENDS += "libcutils"
DEPENDS += "liblog"
DEPENDS += "system-core"

FILES_${PN}-dbg = "${libdir}/.debug"
FILES_${PN}     = "${libdir}/lib*.so.* ${libdir}/lib*.so "
FILES_${PN}-dev = "${libdir}/*.la ${libdir}/lib*.so ${includedir}"

do_install_append() {
        install -d ${D}${includedir}/mm-osal/include
        install -m 0644 ${S}/inc/*.h -D ${D}${includedir}/mm-osal/include/
}
INSANE_SKIP_${PN} += "dev-so"
EXCLUDE_FROM_SHLIBS = "1"
