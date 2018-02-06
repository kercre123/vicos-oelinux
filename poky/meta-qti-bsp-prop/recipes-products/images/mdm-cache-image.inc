# This include forces the generation of a /cache cache partition image file

CACHE_IMAGE_BASENAME = "${MACHINE}-cache"
OUTPUT_CACHE_FILE = "${DEPLOY_DIR_IMAGE}/${CACHE_IMAGE_BASENAME}"
OUTPUT_CACHE_FILE_EXT4 = "${OUTPUT_CACHE_FILE}.ext4"
CACHE_IMAGE_ROOTFS_SIZE = "65536"

# Generating cache image in EXT4 format
IMAGE_PREPROCESS_COMMAND += "create_cache_ext4_img;"

create_cache_ext4_img() {
   if [ "${BASEMACHINE}" != "mdm9607" ]; then
        make_ext4fs -s -l ${CACHE_SIZE_EXT4} ${OUTPUT_CACHE_FILE_EXT4}
        chmod 644 ${OUTPUT_CACHE_FILE_EXT4}
   fi
}
