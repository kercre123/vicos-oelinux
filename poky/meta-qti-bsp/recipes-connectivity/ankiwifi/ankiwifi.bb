DESCRIPTION = "Install default wifi configuration"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI += "file://settings"

do_install_append() {
  install -d ${D}/var/lib/connman
  install -m 644 ${WORKDIR}/settings ${D}/var/lib/connman/
}

FILES_${PN} += "/var/lib/connman"
