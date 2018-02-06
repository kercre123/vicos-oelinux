inherit autotools qcommon qprebuilt
DESCRIPTION = "Neutrino Ethernet Firmware"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/${LICENSE};md5=0835ade698e0bcf8506ecda2f7b4f302"
PR = "r0"

SRC_DIR = "${WORKSPACE}/neutrino-noship/"
S = "${WORKDIR}/neutrino-noship"

FILES_${PN} = "/lib/firmware/DWC_ETH_QOS_fw.bin"

do_install() {
    install -d ${D}/lib/firmware
    install -m 0644 ${S}/firmware_mdm/DWC_ETH_QOS_fw.bin ${D}/lib/firmware/
    install -m 0644 ${S}/NOTICE ${D}/
}
