inherit update-rc.d qcommon qlicense qprebuilt

DESCRIPTION = "QMUXD Module"
PR = "r0"

DEPENDS = "common diag dsutils configdb qmi qmi-framework"

EXTRA_OECONF = "--with-lib-path=${STAGING_LIBDIR} \
                --with-common-includes=${STAGING_INCDIR} \
                --with-qxdm"

EXTRA_OECONF_append_msm8960 = " --enable-auto-answer=yes"

S       = "${WORKDIR}/qmuxd"
SRC_DIR = "${WORKSPACE}/qmi/qmuxd"

CFLAGS += "${CFLAGS_EXTRA}"
CFLAGS_EXTRA_append_arm = " -fforward-propagate"

INITSCRIPT_NAME = "qmuxd"
INITSCRIPT_PARAMS = "start 40 2 3 4 5 . stop 80 0 1 6 ."

do_install_append() {
       install -m 0755 ${WORKDIR}/qmuxd/start_qmuxd_le -D ${D}${sysconfdir}/init.d/qmuxd
       install -m 0755 ${WORKDIR}/qmuxd/qmi_config.xml -D ${D}${sysconfdir}/data/qmi_config.xml
}

INITSCRIPT_PARAMS_MSM8226 = "start 40 2 3 4 5 ."

pkg_postinst_append_msm8226 () {
        [ -n "$D" ] && OPT="-r $D" || OPT="-s"
        # remove all rc.d-links potentially created from alternatives
        update-rc.d $OPT -f ${INITSCRIPT_NAME} remove
        update-rc.d $OPT ${INITSCRIPT_NAME} ${INITSCRIPT_PARAMS_MSM8226}
}
