FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "\
            file://automountsdcard.sh \
"

prefix = ""

do_install_append() {
    # systemd is udev compatible.
    if ${@base_contains('DISTRO_FEATURES','systemd','true','false',d)}; then
        install -d ${D}${sysconfdir}/udev/scripts/
        install -m 0755 ${WORKDIR}/automountsdcard.sh \
            ${D}${sysconfdir}/udev/scripts/automountsdcard.sh
    else
        install -d ${D}${sysconfdir}/mdev
        install -m 0755 ${WORKDIR}/automountsdcard.sh ${D}${sysconfdir}/mdev/
    fi
    mkdir -p ${D}/usr/bin
    ln -s /bin/env ${D}/usr/bin/env
}
