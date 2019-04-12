FILES_${PN} += "/lib/ld-linux-armhf.so.3"

do_install_append() {
    if [ "${MLPREFIX}" == "lib32-" ]; then
        cd ${D}/lib
        ln -sf ld-linux.so.3 ld-linux-armhf.so.3
        cd -
    fi
}
