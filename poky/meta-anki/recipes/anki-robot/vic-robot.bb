DESCRIPTION = "Victor Robot daemon"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SERVICE_FILE = "vic-robot.service"

SRC_URI = "file://${SERVICE_FILE}"
SRC_URI += "file://robot.sudoers"

inherit systemd

do_install_append () {
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${D}${systemd_unitdir}/system/
       install -m 0644 ${WORKDIR}/${SERVICE_FILE} -D ${D}${systemd_unitdir}/system/${SERVICE_FILE}
   fi
   install -m 0750 -d ${D}${sysconfdir}/sudoers.d
   install -m 0644 ${WORKDIR}/robot.sudoers -D ${D}${sysconfdir}/sudoers.d/robot
}

FILES_${PN} += "${sysconfdir}/sudoers.d/robot"
FILES_${PN} += "${systemd_unitdir}/system/"
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"
