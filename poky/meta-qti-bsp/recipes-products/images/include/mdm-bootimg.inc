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
   echo ro.product.name=${ANKI_PRODUCT_NAME} >> ${IMAGE_ROOTFS}/build.prop
   echo ${MACHINE} >> ${IMAGE_ROOTFS}/target

   # Anki-specific version
   ANKI_VERSION=`cat ${IMAGE_ROOTFS}/etc/anki-version`
   RELEASE_VERSION=`cat ${IMAGE_ROOTFS}/etc/version`
   
   echo "ro.anki.version=${ANKI_VERSION}" >> ${IMAGE_ROOTFS}/build.prop

   REV_TAG=""
   GIT=`which git`
   if [ ! -z $GIT ]; then
      REV=`git rev-parse --short HEAD`
      REV_TAG="-${REV}"
      # ro.revision is used by debuggerd to tag tombstone crash dumps
      echo "ro.revision=${REV}" >> ${IMAGE_ROOTFS}/build.prop
   fi

   # ro.build.fingerprint is used by debuggerd to tag tombstone crash dumps
   echo "ro.build.fingerprint=${ANKI_VERSION}-${RELEASE_VERSION}" >> ${IMAGE_ROOTFS}/build.prop

   # standard Android build info

   # build.id can be an alpha-numeric build identifier
   echo "ro.build.id=${ANKI_VERSION}${REV_TAG}-${RELEASE_VERSION}" >> ${IMAGE_ROOTFS}/build.prop

   # build.display.id is intended for displaying a consumer facing build id
   echo "ro.build.display.id=${ANKI_VERSION}" >> ${IMAGE_ROOTFS}/build.prop

   # build.version.incremental is should be an increasing number
   VERSION_CODE=`cat ${IMAGE_ROOTFS}/etc/anki-version-code`
   echo "ro.build.version.incremental=${VERSION_CODE}" >> ${IMAGE_ROOTFS}/build.prop

   echo "ro.build.user=${USER}" >> ${IMAGE_ROOTFS}/build.prop
}
