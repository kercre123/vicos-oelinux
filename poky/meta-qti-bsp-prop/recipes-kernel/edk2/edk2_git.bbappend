DESCRIPTION = "Sign the edk2 bootloader"
LICENSE           = "Qualcomm-Technologies-Inc.-Proprietary"
LIC_FILES_CHKSUM  = "file://${COREBASE}/meta-qti-bsp-prop/files/qcom-licenses/${LICENSE};md5=92b1d0ceea78229551577d4284669bb8"
PR = "r1signed"

FILES_${PN} = "/boot"
SIGNING_TOOLS_DIR = "${TMPDIR}/work-shared/signing_tools"
soc_hw_version = "0x30020000"
soc_vers = "0x3005"
DEPENDS += "gensecimage"
EXTRA_OEMAKE_append += "BOOTLOADER_PLATFORM=msmcobalt"
EXTRA_OEMAKE_append += "SIGNED_KERNEL=1"
EXTRA_OEMAKE_append += "VERIFIED_BOOT_LE=1"
EXTRA_OEMAKE_append += "USER_BUILD_VARIANT=0"
EXTRA_OEMAKE_append += "DEVICE_STATUS=DEFAULT_UNLOCK=true"

# This install overrides the one in base recipe. In this we perform image signing
# using the tools located in signing_tools_dir.
#
do_install() {
    install -d ${D}/boot

    # This performs signing the image at lk/build folder. The final image
    # gets written in the folder image/boot/. Subsequently deploy stage in the
    # main recipe installs to the DEPLOYDIR where rest of the system images are located.
    #
    SECIMAGE_LOCAL_DIR=${SIGNING_TOOLS_DIR}/SecImage \
    USES_SEC_POLICY_MULTIPLE_DEFAULT_SIGN=1 \
    USES_SEC_POLICY_INTEGRITY_CHECK=1 \
    python ${SIGNING_TOOLS_DIR}/SecImage/sectools_builder.py \
        -i ${WORKDIR}/bootable/abl.elf \
        -t ${D}/boot \
        -g abl \
        --soc_hw_version ${soc_hw_version} \
            --soc_vers ${soc_vers} \
            --config=${SIGNING_TOOLS_DIR}/SecImage/config/integration/secimagev2.xml \
        --install_base_dir=${D}/boot \
            > ${S}/secimage.log 2>&1

    install ${D}/boot/abl.elf ${S}

}

