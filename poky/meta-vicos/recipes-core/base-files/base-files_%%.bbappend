FILESEXTRAPATHS_prepend := "${THISDIR}/base-files:"

SRC_URI += "file://99-sysctl.conf"

do_install_append () {
    install -d ${D}/etc/sysctl.d
    install -m 0644 ${WORKDIR}/99-sysctl.conf -D ${D}/${sysconfdir}/sysctl.d/99-sysctl.conf
}

FILES_${PN} += "${sysconfdir}/sysctl.d/99-sysctl.conf"