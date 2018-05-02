# Rootfs creation.

DEPENDS += "lk"

# Formats of root filesystem images.
IMAGE_FSTYPES += "ext4"

IMAGE_INSTALL += "packagegroup-core-boot"

IMAGE_INSTALL += "${ROOTFS_PKGMANAGE_BOOTSTRAP}"
IMAGE_INSTALL += "${CORE_IMAGE_EXTRA_INSTALL}"

# utils and scripts needed for SELinux
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux','packagegroup-core-selinux','',d)}"
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux','selinux-config','',d)}"
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux','refpolicy-mls','',d)}"
IMAGE_INSTALL += "${@base_contains('DISTRO_FEATURES','selinux','start-scripts-misc-daemon','',d)}"

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
IMAGE_INSTALL += "${@base_conditional('USER_BUILD', '1', '', 'system-core-adbd', d)}"
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
IMAGE_EXT4_SELINUX_OPTIONS = "${@base_contains('DISTRO_FEATURES','selinux',' -S ${SELINUX_FILE_CONTEXTS} ','',d)}"

DEPENDS += "cryptsetup-native"

CONFIG_FILE = "${COREBASE}/meta-qti-bsp/files/config_files/anki_user_config"
do_makesystem() {
    echo "================ Creating system ================="
    echo "IMAGE_ROOTFS=" ${IMAGE_ROOTFS}
    if [[ "${DISTRO_FEATURES}" =~ "selinux" ]]; then
        echo "SELINUX_FILE_CONTEXTS=" ${SELINUX_FILE_CONTEXTS}
    fi
    echo "SYSTEM_SIZE_EXT4="  ${SYSTEM_SIZE_EXT4}
    echo "DEPLOY_DIR_IMAGE=" ${DEPLOY_DIR_IMAGE}
    echo "MACHINE=" ${MACHINE}
    echo "SYSTEMRW_SIZE_EXT4=" ${SYSTEMRW_SIZE_EXT4}
    echo "CONFIG_FILE=" ${CONFIG_FILE}
    which make_ext4fs

    RAW_FILE=${DEPLOY_DIR_IMAGE}/${MACHINE}-raw-sysfs.ext4
    SPARSE_FILE=${DEPLOY_DIR_IMAGE}/${MACHINE}-sysfs.ext4
    VERITYSETUP_OUTPUT=${DEPLOY_DIR_IMAGE}/${MACHINE}-veritysetup.txt

    BLOCK_SIZE=4096
    DATA_BLOCKS=$( expr ${SYSTEM_SIZE_EXT4} / $BLOCK_SIZE )

    echo "making the sparse ext4 file system"
    make_ext4fs ${IMAGE_EXT4_EXTRA_OPTIONS} -a / -C ${CONFIG_FILE} ${IMAGE_EXT4_SELINUX_OPTIONS} -l ${SYSTEM_SIZE_EXT4} ${RAW_FILE} ${IMAGE_ROOTFS}
    make_ext4fs -l ${SYSTEMRW_SIZE_EXT4} ${DEPLOY_DIR_IMAGE}/${MACHINE}-systemrw.ext4

    veritysetup --data-block-size=${BLOCK_SIZE} --data-blocks=${DATA_BLOCKS} --hash-offset=${SYSTEM_SIZE_EXT4} format ${RAW_FILE} ${RAW_FILE} >${VERITYSETUP_OUTPUT}
    img2simg ${RAW_FILE} ${SPARSE_FILE}
}
