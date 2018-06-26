do_deploy_append() {
    if [ "${MACHINE}" == "apq8009-robot" ] && [ "${PRODUCT}" == "robot" ]; then
        if [ "${DISTRO}" == "msm-perf" ]; then
            cp -f ${D}/boot/System.map-${KERNEL_VERSION} ${DEPLOY_DIR_IMAGE}/${MACHINE}_${PRODUCT}_perf_System.map
        else
            cp -f ${D}/boot/System.map-${KERNEL_VERSION} ${DEPLOY_DIR_IMAGE}/${MACHINE}_${PRODUCT}_System.map
        fi
    fi
}
