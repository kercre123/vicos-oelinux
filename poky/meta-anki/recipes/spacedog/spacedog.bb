DESCRIPTION = "Spacedog daemon"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

S = "${WORKDIR}"
SYSTEMD_SERVICE_${PN} = "spacedog.service"

SRC_URI = "\
 file://${SYSTEMD_SERVICE_${PN}} \
 file://spacedog.sh \
"

inherit systemd

do_compile () {
}

do_install () {
    install -d ${D}${bindir}
    install -m 0755 spacedog.sh ${D}${bindir}
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_unitdir}/system
        install -m 0644 ${WORKDIR}/${SYSTEMD_SERVICE_${PN}} ${D}${systemd_unitdir}/system
        sed -i -e 's,@BINDIR@,${bindir},g' ${D}${systemd_unitdir}/system/${SYSTEMD_SERVICE_${PN}}
        sed -i -e 's,@DESCRIPTION@,${DESCRIPTION},g' ${D}${systemd_unitdir}/system/${SYSTEMD_SERVICE_${PN}}
    fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
