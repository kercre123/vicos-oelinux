inherit qlicense
DESCRIPTION = "Start up script for secure file system configuration"

SRC_URI +="file://sfs_config"
SRC_URI +="file://sfsconfig.service"

PR = "r2"

INITSCRIPT_NAME = "sfs_config"
INITSCRIPT_PARAMS = "start 80 2 3 4 5 . stop 20 0 1 6 ."

inherit update-rc.d systemd pkgconfig

FILES_${PN} += "${systemd_unitdir}/system/"

do_install_append() {
      if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -m 0755 ${WORKDIR}/sfs_config -D ${D}${sysconfdir}/initscripts/${INITSCRIPT_NAME}
        install -d ${D}/etc/systemd/system/
        install -m 0644 ${WORKDIR}/sfsconfig.service -D ${D}/etc/systemd/system/sfsconfig.service
        install -d ${D}/etc/systemd/system/multi-user.target.wants/
        # enable the service for multi-user.target
        ln -sf /etc/systemd/system/sfsconfig.service \
              ${D}/etc/systemd/system/multi-user.target.wants/sfsconfig.service
      else
        install -m 0755 ${WORKDIR}/sfs_config -D ${D}${sysconfdir}/init.d/${INITSCRIPT_NAME}
      fi
}

