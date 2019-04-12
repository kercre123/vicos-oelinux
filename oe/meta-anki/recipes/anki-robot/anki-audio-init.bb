DESCRIPTION = "Anki ALSA config"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI += "file://anki-audio-init.service"
SRC_URI += "file://anki-audio-init.sh"


inherit systemd

do_install_append() {
  install -d ${D}${sysconfdir}/initscripts/
  install -m 0755 ${WORKDIR}/anki-audio-init.sh ${D}${sysconfdir}/initscripts/anki-audio-init
  install -d ${D}${sysconfdir}/systemd/system/  
  install -m 0644 ${WORKDIR}/anki-audio-init.service \
    -D ${D}${sysconfdir}/systemd/system/anki-audio-init.service
}

SYSTEMD_SERVICE_${PN} = "anki-audio-init.service"
