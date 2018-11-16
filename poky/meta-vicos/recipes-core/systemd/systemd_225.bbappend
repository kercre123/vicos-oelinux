FILESEXTRAPATHS_prepend := "${THISDIR}/systemd:"

SRC_URI += "file://journald.conf"
SRC_URI += "file://system.conf"
SRC_URI += "file://initrd-switch-root.target"
SRC_URI += "file://0001-systemctl-kill-all-units-specified-on-the-command-line.patch"

# Place journald.conf in /etc/systemd/
do_install_append () {
   install -d ${D}/etc/systemd/
   install -m 0644 ${WORKDIR}/journald.conf -D ${D}/etc/systemd/journald.conf
   install -m 0644 ${WORKDIR}/system.conf -D ${D}/etc/systemd/system.conf
   install -m 0644 ${WORKDIR}/initrd-switch-root.target -D ${D}/lib/systemd/system/initrd-switch-root.target

# Fully disable journald as it's replaced by syslog-ng
   rm -rf ${D}/etc/systemd/systemd-journald.service
   rm -rf ${D}/etc/systemd/systemd-journald.socket
   rm -rf ${D}/lib/systemd/system/sockets.target.wants/systemd-journald.socket
   rm -rf ${D}/lib/systemd/system/sockets.target.wants/systemd-journald-audit.socket
   rm -rf ${D}/lib/systemd/system/sockets.target.wants/systemd-journald-dev-log.socket
   rm -rf ${D}/lib/systemd/system/sysinit.target.wants/systemd-journald.service
   rm -rf ${D}/lib/systemd/system/sysinit.target.wants/systemd-journal-flush.service
   rm -rf ${D}/lib/systemd/system/sysinit.target.wants/systemd-journal-catalog-update.service
}

FILES_${PN} += "${sysconfdir}/udev/rules.d/qseecom.rules"
