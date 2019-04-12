DESCRIPTION = "Reboot the robot every day"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS = "systemd"
RDEPENDS_${PN} = "python"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://anki/rebooter/"

S = "${WORKDIR}/anki/rebooter/"
SYSTEM_DIR = "${D}${sysconfdir}/systemd/system"

do_compile() {
}

do_install() {
   mkdir -p ${D}/sbin
   cp ${S}/rebooter.py ${D}/sbin/
   chmod 0755 ${D}/sbin/rebooter.py
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
      install -d ${SYSTEM_DIR}/
      install -d ${SYSTEM_DIR}/multi-user.target.wants/
      install -m 0644 ${S}/rebooter.service -D ${SYSTEM_DIR}/rebooter.service
      install -m 0644 ${S}/rebooter.timer -D ${SYSTEM_DIR}/rebooter.timer
      ln -sf /etc/systemd/system/rebooter.timer ${SYSTEM_DIR}/multi-user.target.wants/rebooter.timer
  fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
