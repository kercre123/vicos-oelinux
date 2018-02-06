FILESEXTRAPATHS_prepend := "${THISDIR}/kernel:"
SRC_URI += "file://qti.key"
SRC_URI += "file://vble-qti.key"
TARGET_SHA_TYPE = "sha256"

do_deploy_append() {
    #Generating signed boot.img
    cd ${DEPLOY_DIR_IMAGE}
    cp ${MACHINE}-boot.img ${MACHINE}-boot.img.nonsecure

    openssl dgst -${TARGET_SHA_TYPE} -binary ${MACHINE}-boot.img.nonsecure > ${MACHINE}-boot.img.${TARGET_SHA_TYPE}
    if ${@bb.utils.contains('DISTRO_FEATURES', 'vble', 'true', 'false', d)}; then
        openssl pkeyutl -sign -in ${MACHINE}-boot.img.${TARGET_SHA_TYPE} -inkey ${WORKDIR}/vble-qti.key -out ${MACHINE}-boot.img.sig -pkeyopt digest:${TARGET_SHA_TYPE} -pkeyopt rsa_padding_mode:pkcs1
    else
        openssl rsautl -sign -in ${MACHINE}-boot.img.${TARGET_SHA_TYPE} -inkey ${WORKDIR}/qti.key -out ${MACHINE}-boot.img.sig
    fi
    dd if=/dev/zero of=${MACHINE}-boot.img.sig.padded bs=2048 count=1
    dd if=${MACHINE}-boot.img.sig of=${MACHINE}-boot.img.sig.padded conv=notrunc
    cat ${MACHINE}-boot.img.nonsecure ${MACHINE}-boot.img.sig.padded > ${MACHINE}-boot.img.secure
    rm -rf ${MACHINE}-boot.img.${TARGET_SHA_TYPE} ${MACHINE}-boot.img.sig ${MACHINE}-boot.img.sig.padded
    mv -f ${MACHINE}-boot.img.secure ${MACHINE}-boot.img

    if [ "${MACHINE}" == "apq8009-robot" ] && [ "${PRODUCT}" == "robot" ]; then
        if [ "${DISTRO}" == "msm-perf" ]; then
            cp -f ${WORKDIR}/build/System.map ${DEPLOY_DIR_IMAGE}/${MACHINE}_${PRODUCT}_perf_System.map
        else
            cp -f ${WORKDIR}/build/System.map ${DEPLOY_DIR_IMAGE}/${MACHINE}_${PRODUCT}_System.map
        fi
    fi
}
