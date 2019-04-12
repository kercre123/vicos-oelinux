inherit qcommon qlicense qprebuilt qsigning

DESCRIPTION = "ms12"
SECTION = "multimedia"
PR = "r0"

DEPENDS = "glib-2.0 libcutils system-media acdbloader tinycompress tinyalsa audio-qaf gensecimage audio-qap-wrapper"

EXTRA_OEMAKE = "DEFAULT_INCLUDES= CPPFLAGS="-I. -I${STAGING_KERNEL_BUILDDIR}/usr/include""

SRC_DIR = "${WORKSPACE}/audio/mm-audio-noship/dolby/ms12/"
S = "${WORKDIR}/audio/mm-audio-noship/dolby/ms12/"
SECIMAGE_BASE= "${SIGNING_TOOLS_DIR}/SecImage"

EXTRA_OECONF = "--with-glib"
EXTRA_OECONF += "--with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
EXTRA_OECONF_append_apq8098 = " MS12_SECURITY_FEATURE_ENABLED=true"
EXTRA_OECONF_append_apq8098 = " AUDIO_FEATURE_ENABLED_QAP=true"

do_install_append() {
    install -d ${D}/data/audio
    $(python ${SECIMAGE_BASE}/sectools_builder.py \
     -i ${D}/${libdir}/libdolby_ms12_wrapper.so \
     -t ${D}/ \
     -g dolby \
     --soc_hw_version=0x30020000 \
     --soc_vers=0x3005 \
     --client_id=0x2 --lib_id=0x1 \
     --build_policy_id=MULTIPLE_DEFAULT_SIGN \
     --config ${SECIMAGE_BASE}/config/integration/secimagev2.xml>${S}/secimage.log 2>&1)

    install ${D}/sign_and_encrypt/default/dolby/libdolby_ms12_wrapper.so ${D}/${libdir}/
    rm -rf ${D}/sign_and_encrypt
    rm -rf ${D}/sign
}

FILES_${PN} += "${userfsdatadir}/*"

FILES_SOLIBSDEV = ""
FILES_${PN} += "${libdir}/*.so"
