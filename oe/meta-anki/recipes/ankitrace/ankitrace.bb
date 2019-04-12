DESCRIPTION = "Anki lttng tracing scripts"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

RDEPEND = "lttng-ust lttng-tools lttng-modules"

SRC_URI += "file://ankitrace.service"
SRC_URI += "file://ankitrace"

do_install_append() {
  install -d ${D}${sysconfdir}/initscripts/
  install -d ${D}${sysconfdir}/systemd/system/
  install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants/
  install -d ${D}/${bindir}/
  install -m 0755 ${WORKDIR}/ankitrace ${D}${bindir}/ankitrace
  install -m 0644 ${WORKDIR}/ankitrace.service -D ${D}${sysconfdir}/systemd/system/ankitrace.service
  ln -sf /etc/systemd/system/ankitrace.service ${D}${sysconfdir}/systemd/system/multi-user.target.wants/ankitrace.service
}
