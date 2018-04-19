inherit qcommon qlicense qprebuilt systemd

DESCRIPTION = "Qualcomm Technologies Data Modules (Excluding ConfigDB and DSUtils)"

PR = "${@base_conditional('PRODUCT', 'psm', 'r26-psm', 'r26', d)}"

DEPENDS = "diag dsutils glib-2.0 qmi qmi-framework qmi-client-helper xmllib virtual/kernel openssl pugixml data-oss libcap"

EXTRA_OECONF = "--with-lib-path=${STAGING_LIBDIR} \
                --with-common-includes=${STAGING_INCDIR} \
                --with-glib \
                --with-qxdm \
                --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include \
                --enable-target=${BASEMACHINE} \
                --with-updated-linux-libc-headers"

EXTRA_OECONF += "${@base_conditional('PRODUCT', 'psm', ' --enable-target-mdm9607psm=yes','', d)}"

FILESPATH =+ "${WORKSPACE}/:"
SRC_URI   = "file://data/"
SRC_URI  += "file://data-init"
SRC_URI  += "file://QCMAP_ConnectionManagerd.service"
SRC_URI  += "file://qtid.service"


S       =  "${WORKDIR}/data"
SRC_DIR =  "${WORKSPACE}/data/"

FILES_${PN}-dbg += " ${prefix}/tests/.debug"
FILES_${PN}-dbg += " ${@base_conditional('BASEPRODUCT', 'robot', '', '/WEBSERVER/www/cgi-bin/.debug', d)}"

FILES_${PN} += " ${prefix}/tests \
                 /lib/systemd/*"
FILES_${PN} += " ${@base_conditional('BASEPRODUCT', 'robot', '', '/WEBSERVER/*', d)}"

BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"

# Runlevels for data daemons in msm targets
pkg_postinst_${PN}_msm () {

	if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'false', 'true', d)}; then
          [ -n "$D" ] && OPT="-r $D" || OPT="-s"
          update-rc.d $OPT -f data-init remove
          update-rc.d $OPT data-init start 97 2 3 4 5 . stop 15 0 1 6 .

          update-rc.d $OPT -f start_QCMAP_ConnectionManager_le remove
          if (test "x${BASEPRODUCT}" != "xdrone"); then
            update-rc.d $OPT start_QCMAP_ConnectionManager_le start 39 2 3 4 5 . stop 61 0 1 6 .
          fi

          update-rc.d $OPT -f start_qti_le remove
          update-rc.d $OPT start_qti_le start 40 2 3 4 5 . stop 60 0 1 6 .
	fi
}

# Runlevels for data daemons in mdm targets
pkg_postinst_${PN}_mdm () {
        [ -n "$D" ] && OPT="-r $D" || OPT="-s"
        update-rc.d $OPT -f netmgrd remove
        update-rc.d $OPT netmgrd start 45 2 3 4 5 . stop 80 0 1 6 .

        update-rc.d $OPT -f data-init remove
        update-rc.d $OPT data-init start 97 2 3 4 5 . stop 15 0 1 6 .

        if [ "${MACHINE}" != "mdm9607" ]; then
          update-rc.d $OPT -f start_ipacm_perf_le remove
          update-rc.d $OPT start_ipacm_perf_le start 37 S . stop 63 0 1 6 .

          update-rc.d $OPT -f start_ipacmdiag_le remove
          update-rc.d $OPT start_ipacmdiag_le start 38 S . stop 62 0 1 6 .
        fi

        if [ ${@d.getVar('PRODUCT', True)} != "psm" ]; then
           update-rc.d $OPT -f start_QCMAP_ConnectionManager_le remove
           update-rc.d $OPT start_QCMAP_ConnectionManager_le start 39 S . stop 61 0 1 6 .

           update-rc.d $OPT -f port_bridge remove
           update-rc.d $OPT port_bridge start 38 S . stop 62 0 1 6 .

           update-rc.d $OPT -f start_qti_le remove
           update-rc.d $OPT start_qti_le start 40 S . stop 60 0 1 6 .

           update-rc.d $OPT -f start_eMBMs_TunnelingModule_le remove
           update-rc.d $OPT start_eMBMs_TunnelingModule_le start 70 2 3 4 5 . stop 30 0 1 6 .

           update-rc.d $OPT -f start_stop_qmi_ip_multiclient remove
           update-rc.d $OPT start_stop_qmi_ip_multiclient start 80 2 3 4 5 . stop 20 0 1 6 .
	fi
}

pkg_postinst_append_mdm9625 () {
        update-rc.d $OPT -f start_QCMAP_Web_CLIENT_le remove
        update-rc.d $OPT start_QCMAP_Web_CLIENT_le start 92 2 3 4 5 . stop 10 0 1 6 .
}

do_install_append_mdm() {
        install -m 0755 ${WORKDIR}/data/netmgr/src/start_netmgrd_le -D ${D}${sysconfdir}/init.d/netmgrd
        install -m 0755 ${WORKDIR}/data/port-bridge/start_port_bridge_le -D ${D}${sysconfdir}/init.d/port_bridge
        if [ "${MACHINE}" != "mdm9607" ]; then
          install -m 0755 ${WORKDIR}/data/ipacm-perf/src/start_ipacm_perf_le -D ${D}${sysconfdir}/init.d/start_ipacm_perf_le
          install -m 0755 ${WORKDIR}/data/qti_socksv5/src/qti_socksv5_conf.xml -D ${D}${userfsdatadir}/qti_socksv5_conf.xml
          install -m 0755 ${WORKDIR}/data/qti_socksv5/src/qti_socksv5_auth.xml -D ${D}${userfsdatadir}/qti_socksv5_auth.xml
          install -m 0755 ${WORKDIR}/data/qti_socksv5/src/qti_socksv5 -D ${D}${bindir}/qti_socksv5
        fi
        install -m 0755 ${WORKDIR}/data/netmgr/src/udhcpc.script -D ${D}${sysconfdir}/udhcpc.d/udhcpc.script
        install -m 0755 ${WORKDIR}/data-init -D ${D}${sysconfdir}/init.d/data-init
        install -m 0755 ${WORKDIR}/data/netmgr/src/netmgr_config.xml -D ${D}${sysconfdir}/data/netmgr_config.xml
        install -m 0755 ${WORKDIR}/data/dsi_netctrl/src/dsi_config.xml -D ${D}${sysconfdir}/data/dsi_config.xml
        install -m 0755 ${WORKDIR}/data/mobileap/server/src/mobileap_cfg.xml -D ${D}${userfsdatadir}/mobileap_cfg.xml
        install -m 0755 ${WORKDIR}/data/mobileap/eMBMS_multiclient/qmi_ip_multiclient/src/qmi_ip_cfg.xml -D ${D}${userfsdatadir}/qmi_ip_cfg.xml
        install -m 0755 ${WORKDIR}/data/mobileap/server/src/l2tp_cfg.xml -D ${D}${userfsdatadir}/l2tp_cfg.xml
}

do_install_append_msm() {
        #data-init,netmgrd,eMBMs_TunnelingModule,qmi_ip_multiclient,ipacm_perf,ipacmdiag,qti_ppp
        #not needed for msm target removing init files
        rm -rf ${D}${sysconfdir}/init.d/data-init ${D}${sysconfdir}/init.d/netmgrd
        rm -rf ${D}${sysconfdir}/init.d/start_eMBMs_TunnelingModule_le ${D}${sysconfdir}/init.d/start_stop_qmi_ip_multiclient
        rm -rf ${D}${sysconfdir}/init.d/start_ipacm_perf_le ${D}${sysconfdir}/init.d/start_ipacmdiag_le
        rm -rf ${D}${sysconfdir}/init.d/start_stop_qti_ppp_le
        rm -rf ${D}${sysconfdir}/init.d/port_bridge

        install -m 0755 ${WORKDIR}/data/mobileap/server/src/mobileap_cfg.xml -D ${D}${userfsdatadir}/mobileap_cfg.xml
        if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
          install -d ${D}${systemd_unitdir}/system/
          #QCMAP service
          if (test "x${BASEPRODUCT}" != "xdrone" && test "x${BASEPRODUCT}" != "xrobot"); then
            rm -rf ${D}${sysconfdir}/init.d/start_QCMAP_ConnectionManager_le
            install -m 0644 ${WORKDIR}/QCMAP_ConnectionManagerd.service -D ${D}${systemd_unitdir}/system/QCMAP_ConnectionManagerd.service
            install -d ${D}${systemd_unitdir}/system/multi-user.target.wants/
            # enable the service for multi-user.target
            ln -sf ${systemd_unitdir}/system/QCMAP_ConnectionManagerd.service \
            ${D}${systemd_unitdir}/system/multi-user.target.wants/QCMAP_ConnectionManagerd.service
          fi

          if (test "x${BASEPRODUCT}" == "xrobot"); then
            rm -rf ${D}/WEBSERVER
          fi

          #QTI service
          rm -rf ${D}${sysconfdir}/init.d/start_qti_le
          install -m 0644 ${WORKDIR}/qtid.service -D ${D}${systemd_unitdir}/system/qtid.service
          install -d ${D}${systemd_unitdir}/system/multi-user.target.wants/
          # enable the service for multi-user.target
          ln -sf ${systemd_unitdir}/system/qtid.service \
          ${D}${systemd_unitdir}/system/multi-user.target.wants/qtid.service

        fi

}
FILES_${PN} += "${userfsdatadir}/mobileap_cfg.xml"
FILES_${PN} += "${userfsdatadir}/qmi_ip_cfg.xml"
FILES_${PN} += "${userfsdatadir}/qti_socksv5_conf.xml"
FILES_${PN} += "${userfsdatadir}/qti_socksv5_auth.xml"
FILES_${PN} += "${userfsdatadir}/l2tp_cfg.xml"
