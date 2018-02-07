EXTRA_OECONF_append=" ${@bb.utils.contains('DISTRO_FEATURES', 'selinux', '--with-selinux', '', d)}"
BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"
do_install_append () {
    if [ "${BASEPRODUCT}" == "drone" ]; then
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
    elif [ "${BASEPRODUCT}" == "robot" ] || [ "${BASEPRODUCT}" == "robot-rome" ]; then
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config_readonly
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config_readonly
        sed -i '$a    StrictHostKeyChecking no' ${WORKDIR}/ssh_config ${D}${sysconfdir}/ssh/ssh_config
        sed -i '$a    UserKnownHostsFile /dev/null' ${WORKDIR}/ssh_config ${D}${sysconfdir}/ssh/ssh_config
    fi
}

RDEPENDS_${PN} += "${PN}-sftp"
