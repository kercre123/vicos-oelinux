# The MSM Linux minimal image rootfs creation files.

# lk supports only 32-bit compilation.
DEPENDS_append_arm += " lk"
DEPENDS_append_aarch64 += " lib32-lk"


IMAGE_INSTALL += "packagegroup-core-boot"
IMAGE_INSTALL += "${ROOTFS_PKGMANAGE_BOOTSTRAP}"
IMAGE_INSTALL += "${CORE_IMAGE_EXTRA_INSTALL}"

IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux', 'packagegroup-core-selinux', '', d)}"
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux', 'selinux-config', '', d)}"
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux', 'refpolicy-mls', '', d)}"

IMAGE_INSTALL += "start-scripts-firmware-links"
IMAGE_INSTALL += "start-scripts-find-partitions"
IMAGE_INSTALL += "start-scripts-misc-daemon"
IMAGE_INSTALL += "start-scripts-update-perms"

IMAGE_INSTALL += "base-files"
IMAGE_INSTALL += "base-passwd"
IMAGE_INSTALL += "dhcpcd"
IMAGE_INSTALL += "native-frameworks"

IMAGE_INSTALL += "glib-2.0"
IMAGE_INSTALL += "libcutils"
IMAGE_INSTALL += "liblog"
IMAGE_INSTALL += "libxml2"
IMAGE_INSTALL += "system-core-adbd"
IMAGE_INSTALL += "system-core-logd"
IMAGE_INSTALL += "system-core-leprop"
IMAGE_INSTALL += "system-core-usb"
IMAGE_INSTALL += "${@base_conditional('USER_BUILD','1','','system-core-debuggerd',d)}"
IMAGE_INSTALL += "system-core-post-boot"
IMAGE_INSTALL += "system-conf"
IMAGE_INSTALL += "system-prop"
IMAGE_INSTALL += "tcpdump"
IMAGE_INSTALL += "wireless-tools"

#32-bit only packages
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-libnl', 'libnl', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-live555-qti', 'live555-qti', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-gtest', 'gtest', d)}"
IMAGE_INSTALL += "${@base_contains('MULTILIB_VARIANTS', 'lib32', 'lib32-cairo', 'cairo', d)}"

IMAGE_INSTALL += "libpagemap"

IMAGE_INSTALL += "libuv"
IMAGE_INSTALL += "libwebsockets"

# Below is to generate sparse ext4 system image (OE by default supports raw ext4 images)
SELINUX_FILE_CONTEXTS = "${IMAGE_ROOTFS}/etc/selinux/mls/contexts/files/file_contexts"

IMAGE_EXT4_SELINUX_OPTIONS = "${@base_contains('DISTRO_FEATURES','selinux',' -a / -S ${SELINUX_FILE_CONTEXTS} ','',d)}"

do_fsconfig_append() {
if [ "${DISTRO_NAME}" == "msm-user" ]; then
sed -i '/mount -t debugfs/ d' ${IMAGE_ROOTFS}/etc/init.d/sysfs.sh
fi
    chmod -R o-rwx ${IMAGE_ROOTFS}/etc/init.d/
}

do_makesystem() {
    cp ${THISDIR}/${BASEMACHINE}/apq8053-fsconfig.conf ${WORKDIR}/rootfs-fsconfig.conf
    make_ext4fs -C ${WORKDIR}/rootfs-fsconfig.conf ${IMAGE_EXT4_EXTRA_OPTIONS} -s ${IMAGE_EXT4_SELINUX_OPTIONS} -l ${SYSTEM_SIZE_EXT4} ${DEPLOY_DIR_IMAGE}/${MACHINE}-sysfs.ext4 ${IMAGE_ROOTFS}
    make_ext4fs  -l ${SYSTEMRW_SIZE_EXT4} ${DEPLOY_DIR_IMAGE}/${MACHINE}-systemrw.ext4
}
