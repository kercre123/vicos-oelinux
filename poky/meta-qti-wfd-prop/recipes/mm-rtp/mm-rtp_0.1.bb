inherit androidmk deploy qlicense qprebuilt

SUMMARY = "beat"
SECTION = "multimedia"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://video/lib/mm-rtp/"
SRC_URI += "file://0001-mm-rtp-Compile-out-external-dependency.patch"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/video/lib/mm-rtp"

SRC_DIR = "${WORKSPACE}/video/lib/mm-rtp"

EXTRA_OEMAKE += "HAS_PREBUILT_PATH=false"

DEPENDS += "mm-osal"
DEPENDS += "media"
DEPENDS += "mm-parser"

#TODO: Alooper it is defined
#TODO: MediaErrors.h please include
CFLAGS += "-DERROR_END_OF_STREAM=-1011"
CFLAGS += "-UANDROID"

CFLAGS += "-I${STAGING_INCDIR}/utils/"
CFLAGS += "-I${STAGING_INCDIR}/mm-osal/include/"
CFLAGS += "-I${STAGING_INCDIR}/mm-rtp/encoder/include/"
CFLAGS += "-I${STAGING_INCDIR}/mm-rtp/decoder/include/"
CFLAGS += "-I${STAGING_INCDIR}/common/inc/"
CFLAGS += "-I${STAGING_INCDIR}/mm-parser/include/"
CFLAGS += "-I${STAGING_KERNEL_DIR}/usr/include"

CFLAGS += "-include Errors.h"
CFLAGS += "-include string.h"

LDFLAGS += "-lcutils"
LDFLAGS += "-lglib-2.0"
LDFLAGS += "-llog"
LDFLAGS += "-lbase"
LDFLAGS += "-lutils"
LDFLAGS += "-lbinder"

do_fixup_before_compile () {
    #
    # remove ALooper.h inclusion
    sed -i '/ALooper/s/^/\/\//' ${S}/decoder/inc/RTPParser.h
	
	# remove thread.h inclusion
	sed -i '/threads\./s/^/\/\//' ${S}/decoder/src/RTPDataSource.cpp
	
	# TBD::remove MediaErrors.h for ERROR_END_OF_STREAM 
	sed -i '/MediaErrors/s/^/\/\//' ${S}/decoder/src/RTPDataSource.cpp
}
addtask fixup_before_compile after do_patch before do_configure

do_compile() {
    # Current support is limited to 32-bit build
    if [ "${MLPREFIX}" == "lib32-" ] || [ "${MLPREFIX}" == "" -a "${TUNE_ARCH}" == "arm" ]; then
        androidmk_setenv
        oe_runmake -f ${LA_COMPAT_DIR}/build/core/main.mk BUILD_MODULES_IN_PATHS=${S} \
            all_modules SHOW_COMMANDS=true || die "make failed"
    else
        die "mm-rtp supports only 32-bit build."
    fi
}

do_install_prepend() {
    install -d ${D}${includedir}/mm-rtp/encoder/include/
	install -d ${D}${includedir}/mm-rtp/decoder/include/
	install -m 0644 ${S}/encoder/inc/RTPEncoder.h -D ${D}${includedir}/mm-rtp/encoder/include/RTPEncoder.h
	install -m 0644 ${S}/encoder/inc/RTCPMessage.h -D ${D}${includedir}/mm-rtp/encoder/include/RTCPMessage.h
	install -m 0644 ${S}/encoder/inc/RTCPReceiver.h -D ${D}${includedir}/mm-rtp/encoder/include/RTCPReceiver.h
	install -m 0644 ${S}/encoder/inc/RTPPacketizer.h -D ${D}${includedir}/mm-rtp/encoder/include/RTPPacketizer.h
	install -m 0644 ${S}/encoder/inc/RTPPacketTransmit.h -D ${D}${includedir}/mm-rtp/encoder/include/RTPPacketTransmit.h
  	install -m 0644 ${S}/decoder/inc/RTPDataSource.h -D ${D}${includedir}/mm-rtp/decoder/include/RTPDataSource.h
  	install -m 0644 ${S}/decoder/inc/RTPParser.h -D ${D}${includedir}/mm-rtp/decoder/include/RTPParser.h
  	install -m 0644 ${S}/decoder/inc/RTPStreamPort.h -D ${D}${includedir}/mm-rtp/decoder/include/RTPStreamPort.h
}
