inherit autotools qcommon qlicense qprebuilt update-rc.d systemd

DESCRIPTION = "GPS Location process launcher"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps/launcher/"
S = "${WORKDIR}/gps/launcher"
DEPENDS = "glib-2.0 qmi-framework loc-base-util gps-utils loc-mq-client loc-pla"
EXTRA_OECONF ="--with-glib"

INITSCRIPT_NAME = "start_loc_launcher"
INITSCRIPT_PARAMS = "start 98 2 3 4 5 . stop 2 0 1 6 ."

SRC_URI +="file://loc_launcher.service"

do_install_append () {

    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -m 0755 ${S}/start_loc_launcher  -D ${D}${sysconfdir}/initscripts/start_loc_launcher
        install -d ${D}${sysconfdir}/systemd/system/
        install -m 0644 ${WORKDIR}/loc_launcher.service -D ${D}${sysconfdir}/systemd/system/loc_launcher.service
        install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants/
        ln -sf /etc/systemd/system/loc_launcher.service \
                          ${D}/etc/systemd/system/multi-user.target.wants/loc_launcher.service
    else
        install -m 0755 ${S}/start_loc_launcher -D ${D}${sysconfdir}/init.d/start_loc_launcher
    fi
}