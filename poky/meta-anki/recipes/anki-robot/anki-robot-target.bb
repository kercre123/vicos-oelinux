DESCRIPTION = "anki-robot.target systemd"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SERVICE_FILE = "anki-robot.target"

SRC_URI = "file://${SERVICE_FILE}"

DEPENDS += "vic-init"
DEPENDS += "vic-robot"
DEPENDS += "vic-anim"
DEPENDS += "vic-engine"
DEPENDS += "vic-cloud"
DEPENDS += "vic-webserver"
DEPENDS += "anki-audio-init"

inherit systemd

do_install_append () {
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_unitdir}/system/
        install -m 0644 ${WORKDIR}/${SERVICE_FILE} -D ${D}${systemd_unitdir}/system/${SERVICE_FILE}
        install -d ${D}${systemd_unitdir}/system/anki-robot.target.wants/

        # create a symlink named victor.target for cli alias
        ln -sf ${systemd_unitdir}/system/${SERVICE_FILE} \
            ${D}${systemd_unitdir}/system/victor.target
   fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"
