DESCRIPTION = "Wipe the robot every shutdown"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS = "systemd"
RDEPENDS_${PN} = "python"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://external/reboottest/"

S = "${WORKDIR}/external/reboottest/"
SYSTEM_DIR = "${D}${sysconfdir}/systemd/system"

do_compile() {
}

do_install() {
   mkdir -p ${D}/sbin
   cp ${S}/reboottest.sh ${D}/sbin/
   chmod 0755 ${D}/sbin/reboottest.sh
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
      install -d ${SYSTEM_DIR}/
      install -d ${SYSTEM_DIR}/multi-user.target.wants/
      install -m 0644 ${S}/reboottest.service -D ${SYSTEM_DIR}/reboottest.service
      install -m 0644 ${S}/reboottest.timer -D ${SYSTEM_DIR}/reboottest.timer
      ln -sf /etc/systemd/system/reboottest.timer ${SYSTEM_DIR}/multi-user.target.wants/reboottest.timer
  fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
