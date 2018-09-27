
FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"
SRC_URI += "file://logrotate.service \
            file://logrotate.timer \
            file://logrotate.conf \
            "

PACKAGECONFIG = " \
  ${@base_contains('DISTRO_FEATURES', 'acl', 'acl', '', d)} \
  ${@base_contains('DISTRO_FEATURES', 'selinux', 'selinux', '', d)} \
  "

CONFFILES_${PN} += "${localstatedir}/lib/logrotate.status \
		    ${sysconfdir}/logrotate.conf"

SERVICE_FILE = "${BPN}.service ${BPN}.timer"

do_install(){
    oe_runmake install DESTDIR=${D} PREFIX=${D} MANDIR=${mandir}
    mkdir -p ${D}${sysconfdir}/logrotate.d
    mkdir -p ${D}${localstatedir}/lib
    install -p -m 644 ${WORKDIR}/logrotate.conf ${D}${sysconfdir}/logrotate.conf
    touch ${D}${localstatedir}/lib/logrotate.status

    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_system_unitdir}
        install -m 0644 ${WORKDIR}/logrotate.service ${D}${systemd_system_unitdir}/logrotate.service
        install -m 0644 ${WORKDIR}/logrotate.timer ${D}${systemd_system_unitdir}/logrotate.timer
        sed -i -e 's,OnCalendar=.*$,OnCalendar=${LOGROTATE_SYSTEMD_TIMER_BASIS},g' ${D}${systemd_system_unitdir}/logrotate.timer
        sed -i -e 's,AccuracySec=.*$,AccuracySec=${LOGROTATE_SYSTEMD_TIMER_ACCURACY},g' ${D}${systemd_system_unitdir}/logrotate.timer
    fi

    if ${@bb.utils.contains('DISTRO_FEATURES', 'sysvinit', 'true', 'false', d)}; then
        mkdir -p ${D}${sysconfdir}/cron.daily
        install -p -m 0755 ${S}/examples/logrotate.cron ${D}${sysconfdir}/cron.daily/logrotate
    fi
}