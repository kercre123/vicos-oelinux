inherit systemd
DESCRIPTION = "Anki Crash Log Service"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"

SRC_URI += "file://anki/anki-crash-log/"
SRC_URI += "file://anki-crash-log.service"
SRC_URI += "file://anki-crash-log.socket"

S = "${WORKDIR}/anki/anki-crash-log"
SYSTEM_DIR = "${D}${sysconfdir}/systemd/system"

do_install() {
  install -d ${D}/bin
  install -m 0755 ${S}/anki-crash-log ${D}/bin/

  if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
    install -d  ${D}${systemd_unitdir}/system/
    install -m 0644 ${WORKDIR}/anki-crash-log.service -D ${D}${systemd_unitdir}/system/
    install -m 0644 ${WORKDIR}/anki-crash-log.socket -D ${D}${systemd_unitdir}/system/

    install -d ${SYSTEM_DIR}/
    install -d ${SYSTEM_DIR}/sockets.target.wants/

    ln -sf ${systemd_unitdir}/system/anki-crash-log.socket \
      ${SYSTEM_DIR}/sockets.target.wants/
  fi   
}

FILES_${PN} += "/bin/anki-crash-log"
FILES_${PN} += "/lib/systemd/system"

RDEPENDS_${PN} += "bash"
