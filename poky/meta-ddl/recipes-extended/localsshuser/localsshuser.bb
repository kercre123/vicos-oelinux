DESCRIPTION = "Anki OTA Engine"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI = "file://localsshuser.sh"

do_install() {
  install -d ${D}${sysconfdir}/init.d
  install -m 0755 ${WORKDIR}/localsshuser.sh ${D}${sysconfdir}/init.d/localsshuser.sh
  install -d ${D}${sysconfdir}/rc3.d
  cd ${D}${sysconfdir}/rc3.d
  ln -s ../init.d/localsshuser.sh S90localsshuser.sh
}

FILES_${PN} += "${sysconfdir}/init.d"