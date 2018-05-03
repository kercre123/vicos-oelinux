SUMMARY = "Small init script directly boots into dm-verity rootfs on emmc."
LICENSE = "Anki-Inc.-Proprietary"
DEPENDS = "virtual/kernel"
RDEPENDS_${PN} = "udev udev-extraconf"
SRC_URI = "file://init-boot.sh"

S = "${WORKDIR}"

do_install() {
        install -m 0755 ${WORKDIR}/init-boot.sh ${D}/init
        install -d ${D}/dev
        mknod -m 622 ${D}/dev/console c 5 1
}

FILES_${PN} += " /init /dev "

# Due to kernel dependency
PACKAGE_ARCH = "${MACHINE_ARCH}"
