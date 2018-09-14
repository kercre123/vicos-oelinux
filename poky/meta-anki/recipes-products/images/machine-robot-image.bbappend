# Camera Open Source packages
include ${BASEMACHINE}/${BASEMACHINE}-anki-robot-image.inc
require include/mdm-bootimg.inc

ROOTFS_POSTPROCESS_COMMAND += '${@bb.utils.contains("IMAGE_FEATURES", "read-only-rootfs", "read_only_robot_rootfs_hook; ", "",d)}'

# A hook function to support read-only-rootfs IMAGE_FEATURES
read_only_robot_rootfs_hook () {
	if [ -e ${IMAGE_ROOTFS}/etc/default/ssh ]; then
		echo 'SYSCONFDIR=/data/ssh' > ${IMAGE_ROOTFS}/etc/default/ssh
	fi
	echo "ro.anki.product.name=${ANKI_PRODUCT_NAME}" >> ${IMAGE_ROOTFS}/build.prop
}
