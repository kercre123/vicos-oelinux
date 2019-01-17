inherit autotools-brokensep pkgconfig systemd

DESCRIPTION = "vl53l1_daemon user space daemon from ST"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

DEPENDS = "stmvl53l1"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://external/vl53l1_daemon/"

EXTRA_OEMAKE = "INSTALL_ROOT=${D}"

SERVICE_FILE = "vl53l1-daemon.service"

S = "${WORKDIR}/external/vl53l1_daemon/"

do_install_append () {
  if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
      install -d ${D}${systemd_unitdir}/system/
      install -m 0644 ${S}${SERVICE_FILE} -D ${D}${systemd_unitdir}/system/${SERVICE_FILE}
      install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants/
      ln -sf ${systemd_unitdir}/system/${SERVICE_FILE} \
              ${D}${sysconfdir}/systemd/system/multi-user.target.wants/${SERVICE_FILE}
  fi
}

FILES_${PN} = "${bindir}/* ${sysconfdir}/*"
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"
