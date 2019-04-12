DESCRIPTION = "Reboot the robot shortly after boot for stress tests"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS = "systemd"
RDEPENDS_${PN} = "python"

SERVICE_FILE = "reboot-stresser.service"
TIMER_FILE = "reboot-stresser.timer"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://anki/reboot-stresser/"

S = "${WORKDIR}/anki/reboot-stresser/"

inherit systemd

do_install_append () {
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${D}${systemd_unitdir}/system/
       for f in ${SERVICE_FILE} ${TIMER_FILE}; do
	   install -m 0644 ${S}/${f} -D ${D}${systemd_unitdir}/system/${f}
       done
   fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
