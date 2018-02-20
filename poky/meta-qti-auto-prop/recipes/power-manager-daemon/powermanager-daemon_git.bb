inherit qcommon qprebuilt qlicense systemd

DESCRIPTION = "Power manager daemon"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI  = "file://vnw/powermanagerdaemon/src/"

SRC_DIR = "${WORKSPACE}/vnw/powermanagerdaemon/src/"

S = "${WORKDIR}/vnw/powermanagerdaemon/src/"

DEPENDS = "glib-2.0 canwrapper-hdr canwrapper powermanager-lib"

EXTRA_OECONF = "--with-locflp-includes=${STAGING_INCDIR}/canwrapper \
                --with-glib"

do_install_append(){
    install -m 0755 ${S}/start_power_manager_daemon -D ${D}${sysconfdir}/init.d/start_power_manager_daemon
}
