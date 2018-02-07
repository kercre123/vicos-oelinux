# Rootfs creation.

DEPENDS += "lk"

# Formats of root filesystem images.
IMAGE_FSTYPES += "ext4"

IMAGE_INSTALL += "packagegroup-core-boot"

IMAGE_INSTALL += "${ROOTFS_PKGMANAGE_BOOTSTRAP}"
IMAGE_INSTALL += "${CORE_IMAGE_EXTRA_INSTALL}"

# utils and scripts needed for SELinux
IMAGE_INSTALL += "packagegroup-core-selinux"
IMAGE_INSTALL += "selinux-config"
IMAGE_INSTALL += "refpolicy-mls"
IMAGE_INSTALL += "start-scripts-misc-daemon"

# The MSM Linux minimal boot image files.


IMAGE_INSTALL += "base-files"
IMAGE_INSTALL += "base-passwd"
IMAGE_INSTALL += "bluez5"

IMAGE_INSTALL += "native-frameworks"

IMAGE_INSTALL += "glib-2.0"

IMAGE_INSTALL += "libnl"
IMAGE_INSTALL += "libcutils"
IMAGE_INSTALL += "liblog"

IMAGE_INSTALL += "libxml2"

IMAGE_INSTALL += "start-scripts-find-partitions"
IMAGE_INSTALL += "start-scripts-firmware-links"
IMAGE_INSTALL += "system-core-adbd"
IMAGE_INSTALL += "system-core-logd"
IMAGE_INSTALL += "system-core-leprop"
IMAGE_INSTALL += "system-core-usb"
IMAGE_INSTALL += "system-core-post-boot"
IMAGE_INSTALL += "system-conf"
IMAGE_INSTALL += "system-prop"
IMAGE_INSTALL += "tcpdump"
IMAGE_INSTALL += "wireless-tools"

#IMAGE_PREPROCESS_COMMAND += "echo ${MACHINE} >> ${IMAGE_ROOTFS}/target;"

# Don't install GDB and related pacakges in production builds.
DEPENDS += "${@base_conditional('USER_BUILD', '1', '', 'gdb-cross-arm', d)}"
IMAGE_INSTALL += "${@base_conditional('USER_BUILD', '1', '', 'gdb', d)}"
IMAGE_INSTALL += "${@base_conditional('USER_BUILD', '1', '', 'gdbserver', d)}"

# Below is to generate sparse ext4 system image (OE by default supports raw ext4 images)
SELINUX_FILE_CONTEXTS = "${IMAGE_ROOTFS}/etc/selinux/mls/contexts/files/file_contexts"
do_makesystem() {

    make_ext4fs -s -a / -S ${SELINUX_FILE_CONTEXTS} -l ${SYSTEM_SIZE_EXT4} ${DEPLOY_DIR_IMAGE}/${MACHINE}-sysfs.ext4 ${IMAGE_ROOTFS}
    make_ext4fs  -l ${SYSTEMRW_SIZE_EXT4} ${DEPLOY_DIR_IMAGE}/${MACHINE}-systemrw.ext4
}
