inherit qcommon qprebuilt qlicense

DESCRIPTION = "omx Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio/omx/"
S = "${WORKDIR}/audio/mm-audio/omx/"

DEPENDS = "media audioalsa glib-2.0 omx-noship"

EXTRA_OECONF += "--with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include \
                 --with-glib \
                 --with-feature-omx-adec-g711"

EXTRA_OEMAKE += "DEFAULT_INCLUDES=-I${WORKSPACE}/hardware/qcom/media/mm-core/inc"

CPPFLAGS_apq8010 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"
CPPFLAGS_apq8037 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"
CPPFLAGS_apq8052 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"
CPPFLAGS_apq8053 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"
CPPFLAGS_apq8084 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"
CPPFLAGS_apq8096 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"
CPPFLAGS_apq8017 += "-DQCOM_AUDIO_USE_SYSTEM_HEAP_ID"

FILES_${PN}-dbg  = "${libdir}/.debug/* ${bindir}/.debug/*"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${bindir}/* ${libdir}/pkgconfig/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"
