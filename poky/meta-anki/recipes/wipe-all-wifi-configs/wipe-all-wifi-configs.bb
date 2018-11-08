DESCRIPTION = "Anki script to wipe all wifi configs from robot"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"


SRC_URI += "file://wipe-all-wifi-configs"
SRC_URI += "file://wipe-all-wifi-configs.sudoers"

do_install_append() {
   install -d ${D}/sbin
   install -m 0700 ${WORKDIR}/wipe-all-wifi-configs ${D}/sbin/wipe-all-wifi-configs
   install -m 0750 -d ${D}${sysconfdir}/sudoers.d
   install -m 0644 ${WORKDIR}/wipe-all-wifi-configs.sudoers -D ${D}${sysconfdir}/sudoers.d/wipe-all-wifi-configs
}

FILES_${PN} += "${systemd_unitdir}/system/"
FILES_${PN} += "${sysconfdir}/sudoers.d/wipe-all-wifi-configs"

