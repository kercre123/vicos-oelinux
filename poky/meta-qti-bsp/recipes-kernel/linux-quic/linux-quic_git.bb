inherit kernel qperf

DESCRIPTION = "QuIC Linux Kernel"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=d7810fab7487fb0aad327b76f1be7cd7"

COMPATIBLE_MACHINE = "(mdm9607|mdm9650|apq8009|apq8096|apq8053|apq8017|msm8909w|sdx20)"

# Default image type is zImage, change it in machine conf if needed.
KERNEL_IMAGETYPE ?= "zImage"

python __anonymous () {
  if (d.getVar('PERF_BUILD', True) == '1'):
      imgtype = d.getVar("KERNEL_PERF_IMAGETYPE", True)
      if imgtype:
          d.setVar("KERNEL_IMAGETYPE", d.getVar("KERNEL_PERF_IMAGETYPE", True))
      perfconf = d.getVar("KERNEL_PERF_DEFCONFIG", True)
      if perfconf:
          d.setVar("KERNEL_CONFIG", d.getVar("KERNEL_PERF_DEFCONFIG", True))
      perfcmd = d.getVar("KERNEL_PERF_CMD_PARAMS", True)
      if perfcmd:
          d.setVar("KERNEL_CMD_PARAMS", d.getVar("KERNEL_PERF_CMD_PARAMS", True))
  else:
      d.setVar("KERNEL_CONFIG", d.getVar("KERNEL_DEFCONFIG", True))

  # Override KERNEL_IMAGETYPE_FOR_MAKE variable, which is internal
  # to kernel.bbclass. We override the variable as msm kernel can't
  # support alternate image builds
  if d.getVar("KERNEL_IMAGETYPE", True):
      d.setVar("KERNEL_IMAGETYPE_FOR_MAKE", "")
}

KERNEL_IMAGEDEST_apq8096 = "boot"

DEPENDS_append_aarch64 = " libgcc"
KERNEL_CC_append_aarch64 = " ${TOOLCHAIN_OPTIONS}"
KERNEL_LD_append_aarch64 = " ${TOOLCHAIN_OPTIONS}"

KERNEL_PRIORITY           = "9001"
# Add V=1 to KERNEL_EXTRA_ARGS for verbose
KERNEL_EXTRA_ARGS        += "O=${B}"

PACKAGE_ARCH = "${MACHINE_ARCH}"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   =  "file://kernel"

SRC_DIR   =  "${WORKSPACE}/kernel/msm-3.18"
S         =  "${WORKDIR}/kernel/msm-3.18"
GITVER    =  "${@base_get_metadata_git_revision('${SRC_DIR}',d)}"
PV = "git"
PR = "${@base_conditional('PRODUCT', 'psm', 'r5-psm', 'r5', d)}"

DEPENDS += "dtbtool-native mkbootimg-native"
DEPENDS_apq8096 += "mkbootimg-native dtc-native"
PACKAGES = "kernel kernel-base kernel-vmlinux kernel-dev kernel-modules"
RDEPENDS_kernel-base = ""

# Put the zImage in the kernel-dev pkg
FILES_kernel-dev += "/${KERNEL_IMAGEDEST}/${KERNEL_IMAGETYPE}-${KERNEL_VERSION}"

# Additional defconfigs for systemd
do_defconfig_patch () {
cat >> ${S}/arch/${ARCH}/configs/${KERNEL_CONFIG} <<KERNEL_EXTRACONFIGS
CONFIG_DEVTMPFS=y
CONFIG_DEVTMPFS_MOUNT=y
CONFIG_FHANDLE=y
KERNEL_EXTRACONFIGS
}

do_patch_append () {
    if bb.utils.contains('DISTRO_FEATURES', 'systemd', True, False, d):
        bb.build.exec_func('do_defconfig_patch',d)
}

do_configure () {
    oe_runmake_call -C ${S} ARCH=${ARCH} ${KERNEL_EXTRA_ARGS} ${KERNEL_CONFIG}
}

do_shared_workdir () {
        cd ${B}

        kerneldir=${STAGING_KERNEL_BUILDDIR}
        install -d $kerneldir

        #
        # Store the kernel version in sysroots for module-base.bbclass
        #

        echo "${KERNEL_VERSION}" > $kerneldir/kernel-abiversion

        # Copy files required for module builds
        cp System.map $kerneldir/System.map-${KERNEL_VERSION}
        cp Module.symvers $kerneldir/Module.symvers
        cp Makefile $kerneldir/
        cp .config $kerneldir/
        cp -fR usr $kerneldir/

        # Signing keys may not be present
        [ -f signing_key.priv ] && cp signing_key.priv $kerneldir/
        [ -f signing_key.x509 ] && cp signing_key.x509 $kerneldir/

        # include/config
        mkdir -p $kerneldir/include/config
        cp include/config/kernel.release $kerneldir/include/config/kernel.release
        cp include/config/auto.conf $kerneldir/include/config/auto.conf

        # We can also copy over all the generated files and avoid special cases
        # like version.h, but we've opted to keep this small until file creep starts
        # to happen
        if [ -e include/linux/version.h ]; then
                mkdir -p $kerneldir/include/linux
                cp include/linux/version.h $kerneldir/include/linux/version.h
        fi

        mkdir -p $kerneldir/include/generated/
        cp -fR include/generated/* $kerneldir/include/generated/

        if [ -d arch/${ARCH}/include ]; then
                mkdir -p $kerneldir/arch/${ARCH}/include/
                cp -fR arch/${ARCH}/include/* $kerneldir/arch/${ARCH}/include/
        fi

        if [ -d arch/${ARCH}/boot ]; then
                mkdir -p $kerneldir/arch/${ARCH}/boot/
                cp -fR arch/${ARCH}/boot/* $kerneldir/arch/${ARCH}/boot/
        fi

        if [ -d scripts ]; then
            for i in \
                scripts/basic/bin2c \
                scripts/basic/fixdep \
                scripts/conmakehash \
                scripts/dtc/dtc \
                scripts/kallsyms \
                scripts/kconfig/conf \
                scripts/mod/mk_elfconfig \
                scripts/mod/modpost \
                scripts/sign-file \
                scripts/sortextable;
            do
                if [ -e $i ]; then
                    mkdir -p $kerneldir/`dirname $i`
                    cp $i $kerneldir/$i
                fi
            done
        fi

        cp ${STAGING_KERNEL_DIR}/scripts/gen_initramfs_list.sh $kerneldir/scripts/

        # Make vmlinux available as soon as possible
        VMLINUX_DIR=${@base_conditional('PERF_BUILD', '1', '${STAGING_DIR_TARGET}-perf', base_conditional('PRODUCT', 'psm', '${STAGING_DIR_TARGET}-psm', '${STAGING_DIR_TARGET}', d), d)}
        install -d ${VMLINUX_DIR}/${KERNEL_IMAGEDEST}
        install -m 0644 ${KERNEL_OUTPUT} ${VMLINUX_DIR}/${KERNEL_IMAGEDEST}/${KERNEL_IMAGETYPE}-${KERNEL_VERSION}
        install -m 0644 vmlinux ${VMLINUX_DIR}/${KERNEL_IMAGEDEST}/vmlinux-${KERNEL_VERSION}
        install -m 0644 vmlinux ${VMLINUX_DIR}/${KERNEL_IMAGEDEST}/vmlinux
}

do_install_append() {
    oe_runmake_call -C ${STAGING_KERNEL_DIR} ARCH=${ARCH} CC="${KERNEL_CC}" LD="${KERNEL_LD}" headers_install O=${STAGING_KERNEL_BUILDDIR}
}

nand_boot_flag = "${@base_contains('DISTRO_FEATURES', 'nand-boot', '1', '0', d)}"

do_deploy[depends] += "machine-robot-image:do_makesystem"

do_deploy_prepend() {

    if [ -f ${D}/${KERNEL_IMAGEDEST}/-${KERNEL_VERSION} ]; then
        mv ${D}/${KERNEL_IMAGEDEST}/-${KERNEL_VERSION} ${D}/${KERNEL_IMAGEDEST}/${KERNEL_IMAGETYPE}-${KERNEL_VERSION}
    fi
    if [ ${nand_boot_flag} == "1" ]; then
        dtb_files=`find ${B}/arch/${ARCH}/boot/dts -iname *${KERNEL_DTS_NAME}*.dtb | awk -Fdts/ '{print $NF}' | awk -F[.][d] '{print $1}'`

        # Create separate images with dtb appended to zImage for all targets.
        for d in ${dtb_files}; do
            #Strip qcom from the result if its present.
            targets=`echo ${d#${KERNEL_DTS_NAME}-}| awk '{split($0,a, "/");print a[2]}'`
            #If dtb are stored inside qcom then we need to search for them inside qcom, else inside dts.
            qcom_check=`echo ${d}| awk '{split($0,a, "/");print a[1]}'`
            if [ ${qcom_check} == "qcom" ]; then
                cat ${D}/${KERNEL_IMAGEDEST}/${KERNEL_IMAGETYPE}-${KERNEL_VERSION} ${B}/arch/${ARCH}/boot/dts/${d}.dtb > ${B}/arch/${ARCH}/boot/dts/qcom/dtb-${KERNEL_IMAGETYPE}-${KERNEL_VERSION}-${targets}
                ${STAGING_BINDIR_NATIVE}/dtbtool ${B}/arch/${ARCH}/boot/dts/qcom/ -s ${PAGE_SIZE} -o ${D}/${KERNEL_IMAGEDEST}/masterDTB -p ${B}/scripts/dtc/ -v
            else
                cat ${D}/${KERNEL_IMAGEDEST}/${KERNEL_IMAGETYPE}-${KERNEL_VERSION} ${B}/arch/${ARCH}/boot/dts/${d}.dtb > ${B}/arch/${ARCH}/boot/dts/dtb-${KERNEL_IMAGETYPE}-${KERNEL_VERSION}-${targets}
                ${STAGING_BINDIR_NATIVE}/dtbtool ${B}/arch/${ARCH}/boot/dts/ -s ${PAGE_SIZE} -o ${D}/${KERNEL_IMAGEDEST}/masterDTB -p ${B}/scripts/dtc/ -v
            fi
        done
    fi
}

do_deploy () {

    extra_mkbootimg_params=""
    if [ ${nand_boot_flag} == "1" ]; then
        extra_mkbootimg_params='--dt ${D}/${KERNEL_IMAGEDEST}/masterDTB --tags-addr ${KERNEL_TAGS_OFFSET}'
    fi

    mkdir -p ${DEPLOY_DIR_IMAGE}

    VERITYSETUP_OUTPUT=${DEPLOY_DIR_IMAGE}/apq8009-robot-veritysetup.txt

    HASH_TYPE=$(sed -n -e '/Hash type:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)
    HASH_ALGORITHM=$(sed -n -e '/Hash algorithm:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)
    DATA_BLOCKS=$(sed -n -e '/Data blocks:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)
    DATA_BLOCK_SIZE=$(sed -n -e '/Data block size:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)
    HASH_BLOCK_SIZE=$(sed -n -e '/Hash block size:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)
    SALT=$(sed -n -e '/Salt:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)
    ROOT_HASH=$(sed -n -e '/Root hash:/ s/.*:\s*// p' $VERITYSETUP_OUTPUT)

    FS_SIZE_IN_512_BLOCKS=$(expr \( $DATA_BLOCKS \* $DATA_BLOCK_SIZE / 512 \) )
    HASH_OFFSET=$(expr \( $DATA_BLOCKS \+ 1 \) )

    ## Where does the magic number `179` below come from?
    ## -> see kernel/msm-3.18/Documentation/devices.txt
    ## -> All MMC block devices get the major device number 179
    ## -> The minor number corresponds to the partition number (counting from 1)
    ## -> We assume the boot partition is always on the first MMC block device
    KERNEL_CMD_PARAMS="${@ d.getVar("KERNEL_CMD_PARAMS") } dm=\"system none ro,0 $FS_SIZE_IN_512_BLOCKS verity $HASH_TYPE 179:##SYSTEM_BOOT_PART## 179:##SYSTEM_BOOT_PART## $DATA_BLOCK_SIZE $HASH_BLOCK_SIZE $DATA_BLOCKS $HASH_OFFSET $HASH_ALGORITHM $ROOT_HASH $SALT\" root=/dev/dm-0"

    ## Save kernel cmd line for debugging purposes
    echo "KERNEL_CMD_PARAMS=$KERNEL_CMD_PARAMS" > ${DEPLOY_DIR_IMAGE}/kernel_params.txt

    ## NOTE: don't use ${KERNEL_CMD_PARAMS} below - it will be expanded to the same what ${@ d.getVar("KERNEL_CMD_PARAMS") } returns
    ##       we are modifying a local shell variable here, that can only be dereferenced by using $KERNEL_CMD_PARAMS
    ##       c.f. https://stackoverflow.com/questions/48213497/how-do-i-update-a-datastore-variable-from-inside-a-bash-variable
    # Make bootimage
    ${STAGING_BINDIR_NATIVE}/mkbootimg --kernel ${D}/${KERNEL_IMAGEDEST}/${KERNEL_IMAGETYPE}-${KERNEL_VERSION} \
        --ramdisk /dev/null \
        --cmdline "$KERNEL_CMD_PARAMS" \
        --pagesize ${PAGE_SIZE} \
        --base ${KERNEL_BASE} \
        --ramdisk_offset 0x0 \
        ${extra_mkbootimg_params} --output ${DEPLOY_DIR_IMAGE}/${MACHINE}-boot.img
}

