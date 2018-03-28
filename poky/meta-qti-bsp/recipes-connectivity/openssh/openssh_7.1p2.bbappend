#
# <anki> 
# VIC-2065: PasswordAuthentication=no
# </anki>
#
FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "file://sshdgenkeys.service"

# EXTRA_OECONF += " --sysconfdir=/data/ssh"

EXTRA_OECONF_append=" ${@bb.utils.contains('DISTRO_FEATURES', 'selinux', '--with-selinux', '', d)}"
BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"
do_install_append () {
    if [ "${BASEPRODUCT}" == "drone" ]; then
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
    elif [ "${BASEPRODUCT}" == "robot" ] || [ "${BASEPRODUCT}" == "robot-rome" ]; then
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication no:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config
        sed -i -e 's:#PermitRootLogin yes:PermitRootLogin yes:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config_readonly
        sed -i -e 's:#PasswordAuthentication yes:PasswordAuthentication no:' ${WORKDIR}/sshd_config ${D}${sysconfdir}/ssh/sshd_config_readonly
        sed -i '$a    StrictHostKeyChecking no' ${WORKDIR}/ssh_config ${D}${sysconfdir}/ssh/ssh_config
        sed -i '$a    UserKnownHostsFile /dev/null' ${WORKDIR}/ssh_config ${D}${sysconfdir}/ssh/ssh_config
    fi
    ln -s /data/ssh/ssh_host_rsa_key ${D}${sysconfdir}/ssh/ssh_host_rsa_key
    ln -s /data/ssh/ssh_host_dsa_key ${D}${sysconfdir}/ssh/ssh_host_dsa_key
    ln -s /data/ssh/ssh_host_ecdsa_key ${D}${sysconfdir}/ssh/ssh_host_ecdsa_key
    ln -s /data/ssh/ssh_host_ed25519_key ${D}${sysconfdir}/ssh/ssh_host_ed25519_key
}

RDEPENDS_${PN} += "${PN}-sftp"
