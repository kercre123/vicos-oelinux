DESCRIPTION = "Anki Automatic Updates"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SERVICE_FILE = "update-engine.service"
TIMER_FILE = "update-engine.timer"

SRC_URI = "file://${SERVICE_FILE}"
SRC_URI += "file://${TIMER_FILE}"

inherit systemd

do_install_append () {
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${D}${systemd_unitdir}/system/
       for f in ${SERVICE_FILE} ${TIMER_FILE}; do
	   install -m 0644 ${WORKDIR}/${f} -D ${D}${systemd_unitdir}/system/${f}
       done
   fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"
SYSTEMD_SERVICE_${PN} = "${TIMER_FILE}"
