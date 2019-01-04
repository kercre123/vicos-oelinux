FILESEXTRAPATHS_prepend := "${THISDIR}/systemd:"

SRC_URI += "file://journald.conf"
SRC_URI += "file://0001-systemctl-kill-all-units-specified-on-the-command-line.patch"
SRC_URI += "file://0001-Set-journal-FILE_SIZE_INCREASE-to-100KB-from-8MB.patch"
SRC_URI += "file://0001-VIC-2911-fixing-problems-with-systemd-tmpfiles-setup.patch"
SRC_URI += "file://0001-VIC-2911-take-out-references-to-systemd-tmpfiles.patch"

# Place journald.conf in /etc/systemd/
do_install_append () {
   install -d ${D}/etc/systemd/
   install -m 0644 ${WORKDIR}/journald.conf -D ${D}/etc/systemd/journald.conf
}

FILES_${PN} += "${sysconfdir}/udev/rules.d/qseecom.rules"
