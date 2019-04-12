# This include forces the generation of a persist image file

PERSIST_IMAGE_BASENAME = "${MACHINE}-persist"
OUTPUT_PERSIST_FILE = "${DEPLOY_DIR_IMAGE}/${PERSIST_IMAGE_BASENAME}.ext4"
PERSIST_IMAGE_ROOTFS_SIZE = "6536668"
PERSIST_DATA_FOLDER = "${TMPDIR}/rootfs/${PERSIST_IMAGE_BASENAME}"
BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"

# Generating persist image in EXT4 format
IMAGE_PREPROCESS_COMMAND += "${@base_contains('DISTRO_FEATURES', 'persist-image', 'create_persist_ext4_img;', '', d)}"

create_persist_ext4_img() {
   mkdir -p ${PERSIST_DATA_FOLDER}
   mkdir -p ${PERSIST_DATA_FOLDER}/sensors
   touch ${PERSIST_DATA_FOLDER}/sensors/sensors_settings
   echo "1" > ${PERSIST_DATA_FOLDER}/sensors/sensors_settings

   if [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_qcom_wlan_nv.bin" ];then
       if [ "${BASEPRODUCT}" == "robot" ] && [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_qcom_wlan_nv_8909_robot.bin" ]; then
           cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_qcom_wlan_nv_8909_robot.bin" ${PERSIST_DATA_FOLDER}/WCNSS_qcom_wlan_nv.bin
       else
           cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_qcom_wlan_nv.bin" ${PERSIST_DATA_FOLDER}
       fi
   fi

   if [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_wlan_dictionary.dat" ]; then
       cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_wlan_dictionary.dat" ${PERSIST_DATA_FOLDER}
   elif [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}_32/WCNSS_wlan_dictionary.dat" ]; then
       cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}_32/WCNSS_wlan_dictionary.dat" ${PERSIST_DATA_FOLDER}
   fi

   make_ext4fs -s -l ${PERSIST_IMAGE_ROOTFS_SIZE} ${OUTPUT_PERSIST_FILE} ${PERSIST_DATA_FOLDER}
   chmod 644 ${OUTPUT_PERSIST_FILE}
}

PERSIST_UBINIZE_CFG              = "${DEPLOY_DIR_IMAGE}/ubinize_persist.cfg"
PERSIST_IMAGE_BASENAME       = "${MACHINE}-persist"
PERSIST_DATA_ROOTFS          = "${TMPDIR}/rootfs/${PERSIST_IMAGE_BASENAME}"
OUTPUT_FILE_PERSIST_UBIFS    = "${DEPLOY_DIR_IMAGE}/${PERSIST_IMAGE_BASENAME}.ubifs"
OUTPUT_FILE_PERSIST_UBI      = "${DEPLOY_DIR_IMAGE}/${PERSIST_IMAGE_BASENAME}.ubi"

IMAGE_PREPROCESS_COMMAND += "${@base_contains('DISTRO_FEATURES', 'nand-boot', 'create_persist_ubi_img;', '', d)}"

create_persist_ubi_img() {
   if [ "${BASEMACHINE}" == "apq8009" ]; then
       if [ "${BASEPRODUCT}" == "robot" ] || [ "${BASEPRODUCT}" == "robot-rome" ]; then
           create_persist_ubi_rootfs
           create_ubinize_persist_config
           mkfs.ubifs -r ${PERSIST_DATA_ROOTFS} -o ${OUTPUT_FILE_PERSIST_UBIFS} ${MKUBIFS_PERSIST_ARGS}
           ubinize -o ${OUTPUT_FILE_PERSIST_UBI} ${UBINIZE_ARGS} ${PERSIST_UBINIZE_CFG}
           chmod 644 ${OUTPUT_FILE_PERSIST_UBIFS}
       fi
   fi
}

create_ubinize_persist_config() {
   echo \[persist_volume\] >> "${PERSIST_UBINIZE_CFG}"
   echo mode=ubi >> "${PERSIST_UBINIZE_CFG}"
   echo image="${OUTPUT_FILE_PERSIST_UBIFS}" >> "${PERSIST_UBINIZE_CFG}"
   echo vol_id=0 >> "${PERSIST_UBINIZE_CFG}"
   echo vol_type=dynamic >> "${PERSIST_UBINIZE_CFG}"
   echo vol_name=persist >> "${PERSIST_UBINIZE_CFG}"
   echo vol_size="${PERSIST_VOLUME_SIZE}" >> "${PERSIST_UBINIZE_CFG}"
}

create_persist_ubi_rootfs() {
   mkdir -p ${PERSIST_DATA_ROOTFS}

   if [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_qcom_wlan_nv.bin" ];then
       cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_qcom_wlan_nv.bin" ${PERSIST_DATA_ROOTFS}
   fi

   if [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_wlan_dictionary.dat" ]; then
       cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}/WCNSS_wlan_dictionary.dat" ${PERSIST_DATA_ROOTFS}
   elif [ -e "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}_32/WCNSS_wlan_dictionary.dat" ]; then
       cp -rf "${WORKSPACE}/android_compat/device/qcom/${SOC_FAMILY}_32/WCNSS_wlan_dictionary.dat" ${PERSIST_DATA_ROOTFS}
   fi
}
