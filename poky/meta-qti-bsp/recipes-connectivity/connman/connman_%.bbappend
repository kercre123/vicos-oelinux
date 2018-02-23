FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "file://connman \
	file://connman-pre.service"

EXTRA_OECONF += "--localstatedir=/data"
SYSTEMD_SERVICE_${PN} += " connman-pre.service"

do_install_append () {
	install -d ${D}${sysconfdir}/default
	install -m 0755 ${WORKDIR}/connman ${D}${sysconfdir}/default/
	if ${@bb.utils.contains('DISTRO_FEATURES','systemd','true','false',d)}; then
		install -d ${D}${base_libdir}/systemd/system/
		install -m 0644 ${WORKDIR}/connman-pre.service \
			-D ${D}${base_libdir}/systemd/system/connman-pre.service
	fi
}
