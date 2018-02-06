inherit autotools qcommon qlicense
DESCRIPTION = "Video PostProcessing Library"
SECTION = "multimedia"
PR = "r0"

DEPENDS = "glib-2.0 liblog libcutils vpp-noship adsprpc"

FILESPATH =+ "${WORKSPACE}/video/lib/:"
SRC_URI = "file://vpp/"
SRC_DIR = "${WORKSPACE}/video/lib/vpp/"
S = "${WORKDIR}/vpp/"

EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/ "
EXTRA_OECONF_append =" --with-vppnoship-headers=${STAGING_INCDIR}/vpp-noship/ "
EXTRA_OECONF_append =" --with-glib "
EXTRA_OECONF_append =" --enable-target=${BASEMACHINE}"
EXTRA_OECONF_append =" --enable-vpp-lib=yes "
EXTRA_OECONF_append ="${@base_conditional('MLPREFIX', 'lib32-', '--enable-vpp-test=yes', '', d)}"

do_install_append() {
  install -d ${D}${includedir}/vpp/
  install -m 0644 ${S}/inc/*.h -D ${D}${includedir}/vpp/
}
FILES_${PN}-dbg = "${libdir}/.debug/* ${bindir}/.debug/*"
FILES_${PN} += "${libdir}/*.so ${bindir}/*"
FILES_${PN}-dev = "${libdir}/*.la ${includedir}"
INSANE_SKIP_${PN} = "dev-deps"
