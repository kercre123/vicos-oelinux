inherit autotools-brokensep pkgconfig

DESCRIPTION = "vl53l1 test tools"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS = "stmvl53l1 vl53l1-daemon"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://external/vl53l1_tools/"

EXTRA_OEMAKE = "INSTALL_ROOT=${D}"

S = "${WORKDIR}/external/vl53l1_tools/"

do_install() {
  install -d ${D}/usr/bin
  install -m 0755 ${S}/phio ${D}/usr/bin/
  install -m 0755 ${S}/vl53l1_reg ${D}/usr/bin/
  install -m 0755 ${S}/vl53l1_iotest ${D}/usr/bin/
  install -m 0755 ${S}/ankitof ${D}/usr/bin/
}

FILES_${PN} = "/usr/bin/vl53l1_reg /usr/bin/vl53l1_iotest /usr/bin/phio /usr/bin/ankitof"
