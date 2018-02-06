inherit autotools-brokensep module update-rc.d qperf
DESCRIPTION = "Embms Kernel Module"
LICENSE = "ISC"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta/files/common-licenses/${LICENSE};md5=f3b90e78ea0cffb20bf5cca7947a896d"

PR = "${@base_conditional('PRODUCT', 'psm', 'r0-psm', 'r0', d)}"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://kernel/msm-3.18/net/embms_kernel/ \
           file://start_embms_le"

S = "${WORKDIR}/kernel/msm-3.18/net/embms_kernel/"

FILES_${PN}="/etc/init.d/start_embms_le"

do_install() {
    module_do_install
    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/start_embms_le ${D}${sysconfdir}/init.d
}

INITSCRIPT_NAME = "start_embms_le"
INITSCRIPT_PARAMS = "start 35 5 . stop 15 0 1 6 ."
