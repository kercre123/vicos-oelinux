SUMMARY = "Small init script directly boots into dm-verity rootfs on emmc."
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"
DEPENDS = "virtual/kernel"
RDEPENDS_${PN} = "udev udev-extraconf"
SRC_URI = "file://init-boot.sh file://syscon.dfu"

S = "${WORKDIR}"

do_install() {
        install -m 0755 ${WORKDIR}/init-boot.sh ${D}/init
        install -m 0644 ${WORKDIR}/syscon.dfu ${D}
        install -d ${D}/dev
        mknod -m 622 ${D}/dev/console c 5 1
}

FILES_${PN} += " /init /dev unlock.list syscon.dfu"

# Due to kernel dependency
PACKAGE_ARCH = "${MACHINE_ARCH}"
