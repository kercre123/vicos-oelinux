FILESEXTRAPATHS_prepend := "${THISDIR}/files:"
DEPENDS = "data libnfnetlink"
SRC_URI += "\
    file://0002-qcmap-enabled.patch \
"

do_install_append() {
              rm -rf ${D}${sysconfdir}/miniupnpd/
              install -d ${D}${userfsdatadir}/miniupnpd
              install -m 644 ${WORKDIR}/${PN}-${PV}/miniupnpd.conf ${D}${userfsdatadir}/miniupnpd
}
FILES_${PN} += "${userfsdatadir}/miniupnpd/miniupnpd.conf"
