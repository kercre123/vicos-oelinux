DESCRIPTION = "Anki Robot Early Boot Self Test"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"


SRC_URI = "file://rampost \
           file://rampost.service \
           "

do_install() {
  install -d ${D}/bin
  install -m 0755 ${WORKDIR}/rampost ${D}/bin/

  install -d  ${D}${systemd_unitdir}/system/
  install -m 0644 ${WORKDIR}/rampost.service -D ${D}${sysconfdir}/system/rampost.service
  install -d ${D}${sysconfdir}/system/sysinit.target.wants/
  # enable the service for sysinit.target
  ln -sf ${sysconfdir}/system/rampost.service \
       ${D}${sysconfdir}/system/sysinit.target.wants/rampost.service
}

FILES_${PN} += "/bin"
FILES_${PN} += "/etc/systemd/system"
