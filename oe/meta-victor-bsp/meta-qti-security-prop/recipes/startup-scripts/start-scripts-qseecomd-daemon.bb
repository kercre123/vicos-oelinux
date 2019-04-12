inherit qlicense
DESCRIPTION = "Start up script for securemsm qseecomd daemon"

SRC_URI +="file://start_qseecomd-daemon"
SRC_URI +="file://qseecomd.service"

PR = "r3"

INITSCRIPT_NAME = "qseecomd"
INITSCRIPT_PARAMS = "start 8 2 3 4 5 . stop 20 0 1 6 ."

inherit update-rc.d systemd pkgconfig

FILES_${PN} += "${systemd_unitdir}/system/"

do_install_append() {
       if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -m 0755 ${WORKDIR}/start_qseecomd-daemon -D ${D}${sysconfdir}/initscripts/${INITSCRIPT_NAME}
        install -d ${D}/etc/systemd/system/
        install -m 0644 ${WORKDIR}/qseecomd.service -D ${D}/etc/systemd/system/qseecomd.service
        install -d ${D}/etc/systemd/system/multi-user.target.wants/
        # enable the service for multi-user.target
        ln -sf /etc/systemd/system/qseecomd.service \
              ${D}/etc/systemd/system/multi-user.target.wants/qseecomd.service
       else
        install -m 0755 ${WORKDIR}/start_qseecomd-daemon -D ${D}${sysconfdir}/init.d/${INITSCRIPT_NAME}
       fi
}
