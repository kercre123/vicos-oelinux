DESCRIPTION = "Anki post boot init scripts"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS = "emr-cat"

SRC_URI += "file://ankiinit.service"
SRC_URI += "file://ankiinit.sh"

do_install_append() {
  install -d ${D}${sysconfdir}/initscripts/
  install -m 0755 ${WORKDIR}/ankiinit.sh ${D}${sysconfdir}/initscripts/ankiinit
  install -d ${D}${sysconfdir}/systemd/system/
  install -m 0644 ${WORKDIR}/ankiinit.service \
    -D ${D}${sysconfdir}/systemd/system/ankiinit.service
  install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants/
  ln -sf /etc/systemd/system/ankiinit.service \
    ${D}${sysconfdir}/systemd/system/multi-user.target.wants/ankiinit.service
}
