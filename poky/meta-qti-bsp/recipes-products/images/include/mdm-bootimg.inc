# Force a few additional dependencies in the mix so that we get the needed
# recipes to build in the right order so we can make the bootimg file and
# our images...
DEPENDS += " \
             virtual/kernel \
             pkgconfig-native \
             gtk-doc-native \
             gettext-native \
             e2fsprogs-native \
             ext4-utils-native \
             mtd-utils-native \
"

IMAGE_LINGUAS = ""

# Default image output types
IMAGE_FSTYPES ?= "tar.gz ${INITRAMFS_FSTYPES}"

# Use busybox as login manager
IMAGE_LOGIN_MANAGER = "busybox-static"

# Include minimum init and init scripts
IMAGE_DEV_MANAGER ?= "busybox-static-mdev"
IMAGE_INIT_MANAGER ?= "sysvinit sysvinit-pidof"
IMAGE_INITSCRIPTS ?= ""

IMAGE_PREPROCESS_COMMAND_prepend = " gen_buildprop;"
EXTRA_IMAGE_FEATURES += "${@base_contains('DISTRO_FEATURES','ro-rootfs','read-only-rootfs','',d)}"
gen_buildprop() {
   mkdir -p ${IMAGE_ROOTFS}/cache
   echo ro.build.version.release=`cat ${IMAGE_ROOTFS}/etc/version ` >> ${IMAGE_ROOTFS}/build.prop
   echo ro.product.name=${BASEMACHINE}-${PRODUCT} >> ${IMAGE_ROOTFS}/build.prop
   echo ${MACHINE} >> ${IMAGE_ROOTFS}/target
}
