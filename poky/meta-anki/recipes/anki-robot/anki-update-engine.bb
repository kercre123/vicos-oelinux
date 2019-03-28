DESCRIPTION = "Anki Automatic Updates"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS += "systemd"

SERVICE_FILE = "update-engine.service"

SRC_URI = "file://${SERVICE_FILE}"

SYSTEM_DIR = "${D}${systemd_system_unitdir}"

do_install_append () {
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${SYSTEM_DIR}/
       install -m 0644 ${WORKDIR}/${SERVICE_FILE} -D ${SYSTEM_DIR}/${SERVICE_FILE}
   fi
}

FILES_${PN} += "${systemd_system_unitdir}"
