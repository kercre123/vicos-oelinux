inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "Video PostProcessing No-Ship Library"
SECTION = "multimedia"
PR = "r0"

DEPENDS = "glib-2.0 liblog libcutils adsprpc"

FILESPATH =+ "${WORKSPACE}/video/lib:"
SRC_URI = "file://vpp-noship/"
SRC_DIR = "${WORKSPACE}/video/lib/vpp-noship/"
S = "${WORKDIR}/vpp-noship/"

PREBUILT = "1"

EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/ "
EXTRA_OECONF_append =" --enable-target=${BASEMACHINE}"

FILES_${PN} +="${@base_conditional('BASEMACHINE', 'apq8098', '${libdir}/libvpphvx.so*', '', d)}"
FILES_${PN} +="${@base_conditional('BASEMACHINE', 'apq8098', '/usr/lib/rfsa/adsp/*.so', '', d)}"

do_install_append() {
  install -d ${D}${includedir}/vpp-noship/
  install -m 0644 ${S}/inc/*.h -D ${D}${includedir}/vpp-noship/
}

do_install_append_apq8098() {
  install -m 0644 ${S}/hvx/msm8998/inc/*.h -D ${D}${includedir}/vpp-noship/
  if [ "${MLPREFIX}" == "lib32-" ] || [ "${MLPREFIX}" == "" -a "${TUNE_ARCH}" == "arm" ]; then
    install -d ${D}${libdir}/rfsa/adsp
    install -m 0644 ${S}/hvx/msm8998/prebuilt/*.so -D ${D}${libdir}/rfsa/adsp/
  fi
}
FILES_${PN}-dbg = "${libdir}/.debug/*"
FILES_${PN} += "${libdir}/*.so ${includedir}/vpp-noship/*.h"
FILES_${PN}-dev = "${libdir}/*.la"
INSANE_SKIP_${PN} = "arch dev-deps already-stripped"
