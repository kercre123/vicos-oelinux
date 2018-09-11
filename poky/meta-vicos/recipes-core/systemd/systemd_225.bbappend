FILESEXTRAPATHS_prepend := "${THISDIR}/systemd:"

SRC_URI += "file://journald.conf"

# Place journald.conf in /etc/systemd/
do_install_append () {
   install -d ${D}/etc/systemd/
   install -m 0644 ${WORKDIR}/journald.conf -D ${D}/etc/systemd/journald.conf
}

FILES_${PN} += "${sysconfdir}/udev/rules.d/qseecom.rules"
