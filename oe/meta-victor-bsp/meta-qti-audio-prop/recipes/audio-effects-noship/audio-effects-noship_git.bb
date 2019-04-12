inherit qcommon qprebuilt qlicense

DESCRIPTION = "audio-effects Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio-noship/audio-effects/"
S = "${WORKDIR}/audio/mm-audio-noship/audio-effects/"

DEPENDS = "libcutils system-media"

EXTRA_OECONF += "BOARD_USES_SRS_TRUEMEDIA=false"
EXTRA_OECONF += "MM_AUDIO_ENABLED_SAFX=true"
EXTRA_OECONF += "AUDIO_FEATURE_ENABLED_AUDIOSPHERE=true"

do_install_append () {
  mv ${D}/${libdir}/libsafx_bassboost.a ${D}/${libdir}/libsafx-bassboost.a
  mv ${D}/${libdir}/libsafx_csim.a ${D}/${libdir}/libsafx-csim.a
  mv ${D}/${libdir}/libsafx_pp.a ${D}/${libdir}/libsafx-pp.a
  mv ${D}/${libdir}/libsafx_eq.a ${D}/${libdir}/libsafx-eq.a
  mv ${D}/${libdir}/libsafx_pbe.a ${D}/${libdir}/libsafx-pbe.a
  mv ${D}/${libdir}/libsafx_reverb.a ${D}/${libdir}/libsafx-reverb.a
  mv ${D}/${libdir}/libsafx_virt.a ${D}/${libdir}/libsafx-virt.a
}

FILES_${PN}-dbg  = "${libdir}/.debug/*"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${libdir}/pkgconfig/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"
