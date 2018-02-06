inherit  qcommon qlicense qprebuilt

DESCRIPTION = "drc Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio-noship/drc/"
S = "${WORKDIR}/audio/mm-audio-noship/drc/"

FILES_${PN}-dbg  = "${libdir}/.debug/*"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${libdir}/pkgconfig/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"

do_install_append () {
  install -d ${D}${sysconfdir}
  install -m 0755 ${S}/cfg/* ${D}${sysconfdir}/
}
