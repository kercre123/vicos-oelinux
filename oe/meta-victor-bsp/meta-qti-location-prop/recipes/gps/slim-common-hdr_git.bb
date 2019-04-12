inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "slim common hdr"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/slim/common/"
S = "${WORKDIR}/gps-noship/slim/common"
do_configure() {
}

do_compile() {
}

do_install() {
if [ -d "${SRC_DIR}" ]; then
    install -d ${D}/usr/include
    install -d ${D}/usr/include/libslimcommon
    install -m 644 ${S}/osal/inc/*.h ${D}/usr/include/libslimcommon
    install -m 644 ${S}/core/inc/*.h ${D}/usr/include/libslimcommon
    install -m 644 ${S}/client/inc/*.h ${D}/usr/include/libslimcommon
else
    qprebuilt_do_install

fi

}
