DESCRIPTION = "vl53l1 user space driver"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://anki/vl53l1/"

EXTRA_OEMAKE = "INSTALL_ROOT=${D}"

S = "${WORKDIR}/anki/vl53l1/"

do_install() {
  install -d ${D}/usr/bin
  install -m 0755 ${S}/vl53l1_platform_test ${D}/usr/bin/
  install -m 0755 ${S}/vl53l1_mz_test ${D}/usr/bin/
}

FILES_${PN} = "/usr/bin/vl53l1_platform_test /usr/bin/vl53l1_mz_test"
