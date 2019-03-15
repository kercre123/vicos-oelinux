FILESEXTRAPATHS_prepend := "${THISDIR}/systemd:"

SRC_URI += "file://journald.conf \
            file://0001-VIC-2911-do-not-attempt-to-remove-etc-mtab.patch \
            file://0001-systemctl-kill-all-units-specified-on-the-command-line.patch \
            file://0001-Set-journal-FILE_SIZE_INCREASE-to-100KB-from-8MB.patch \
            file://0001-VIC-2911-fixing-problems-with-systemd-tmpfiles-setup.patch \
            file://0001-VIC-2911-take-out-references-to-systemd-tmpfiles.patch \
            file://0001-Prevent-timer-looping-when-unit-cannot-start.patch \
           "

# Place journald.conf in /etc/systemd/
do_install_append () {
   install -d ${D}/etc/systemd/
   install -m 0644 ${WORKDIR}/journald.conf -D ${D}/etc/systemd/journald.conf
}

FILES_${PN} += "${sysconfdir}/udev/rules.d/qseecom.rules"
