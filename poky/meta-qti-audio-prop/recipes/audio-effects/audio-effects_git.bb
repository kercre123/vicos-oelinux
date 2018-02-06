inherit qcommon qlicense qprebuilt

DESCRIPTION = "audio-effects Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio/audio-effects/"
S = "${WORKDIR}/audio/mm-audio/audio-effects/"

DEPENDS = "virtual/kernel system-media"

EXTRA_OECONF_append = "--with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"

do_install_append() {
    install -m 0755 ${S}/srs/TruMedia/* -D ${D}${includedir}/audio-effects/
}

FILES_${PN}-dbg  = "${libdir}/.debug/*"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${libdir}/pkgconfig/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"
