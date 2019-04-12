inherit qcommon qlicense qprebuilt

DESCRIPTION = "K61 firmware"
PR = "r1"

SRC_DIR = "${WORKSPACE}/vnw/"
S = "${WORKDIR}/vnw"

do_install() {
    install -d ${D}${sysconfdir}/firmware
    install -m 644 ${S}/firmware/K61_firmware.elf ${D}${sysconfdir}/firmware
}
