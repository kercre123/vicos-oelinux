DESCRIPTION = "httpsdate sets the clock from an https server"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS += "systemd"
DEPENDS += "busybox"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://anki/httpsdate/"
SERVICE_FILE = "httpsdate-oneshot.service"

S = "${WORKDIR}/anki/httpsdate/"
SYSTEM_DIR = "${D}${sysconfdir}/systemd/system"

do_compile() {
}

do_install() {
   mkdir -p ${D}/sbin
   cp ${S}/httpsdate ${D}/sbin/
   chmod 0755 ${D}/sbin/httpsdate
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
      install -d ${SYSTEM_DIR}/
      install -d ${SYSTEM_DIR}/multi-user.target.wants/
      install -m 0644 ${S}/${SERVICE_FILE} -D ${SYSTEM_DIR}/${SERVICE_FILE}
      ln -sf /etc/systemd/system/${SERVICE_FILE} ${SYSTEM_DIR}/multi-user.target.wants/${SERVICE_FILE}
   fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
