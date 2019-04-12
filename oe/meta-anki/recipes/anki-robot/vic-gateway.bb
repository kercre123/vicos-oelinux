DESCRIPTION = "Victor gateway daemon"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SERVICE_FILE = "vic-gateway.service"
AVAHI_SERVICE_FILE = "vector-mdns.service"

SRC_URI = "file://${SERVICE_FILE}"
SRC_URI += "file://${AVAHI_SERVICE_FILE}"

inherit systemd

do_install_append () {
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${D}${systemd_unitdir}/system/
       install -m 0644 ${WORKDIR}/${SERVICE_FILE} -D ${D}${systemd_unitdir}/system/${SERVICE_FILE}
   fi

   if ${@bb.utils.contains('DISTRO_FEATURES', 'avahi', 'true', 'false', d)}; then
       install -d ${D}${sysconfdir}/avahi/services/
       install -m 644 ${WORKDIR}/${AVAHI_SERVICE_FILE} ${D}${sysconfdir}/avahi/services/${AVAHI_SERVICE_FILE}
   fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"
