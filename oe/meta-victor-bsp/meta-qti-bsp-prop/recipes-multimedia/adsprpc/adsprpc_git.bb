inherit qcommon qlicense qprebuilt

SUMMARY = "adsprpc daemon"

DEPENDS += "system-core"

FILESPATH =+ "${WORKSPACE}/:"
SRC_URI   = "file://adsprpc"
SRC_URI  += "file://start_adsprpcd"
SRC_URI  += "file://start_sdsprpcd"
SRC_URI  += "file://start_mdsprpcd"
SRC_URI  += "file://adsprpcd.service"
SRC_URI  += "file://sdsprpcd.service"
SRC_URI  += "file://mdsprpcd.service"

SRC_DIR   = "${WORKSPACE}/adsprpc"

S  = "${WORKDIR}/adsprpc"
PR = "r1"

EXTRA_OECONF_apq8096 += "--enable-sdsprpc"
EXTRA_OECONF_apq8009 += "--enable-mdsprpc"

INITSCRIPT_PACKAGES         = "${PN}"
INITSCRIPT_PACKAGES_apq8096 = "${PN} ${PN}-sdsp"

INITSCRIPT_NAME_${PN}         = "adsprpcd"
INITSCRIPT_NAME_${PN}_apq8009 = "mdsprpcd"
INITSCRIPT_PARAMS_${PN}       = "start 70 2 3 4 5 S . stop 30 0 1 6 ."
INITSCRIPT_NAME_${PN}-sdsp    = "sdsprpcd"
INITSCRIPT_PARAMS_${PN}-sdsp  = "start 70 2 3 4 5 . stop 69 0 1 6 ."

inherit update-rc.d systemd pkgconfig

SYSTEMD_SERVICE_${PN}          = "${INITSCRIPT_NAME_${PN}}.service"
SYSTEMD_SERVICE_${PN}_apq8096 += "sdsprpcd.service"

do_install_append () {
    install -m 0755 ${WORKDIR}/start_${INITSCRIPT_NAME_${PN}} -D ${D}${sysconfdir}/init.d/${INITSCRIPT_NAME_${PN}}

    # Install systemd unit files
    install -d ${D}${systemd_unitdir}/system
    install -m 0644 ${WORKDIR}/${INITSCRIPT_NAME_${PN}}.service ${D}${systemd_unitdir}/system
}

do_install_append_apq8096 () {
    install -m 0755 ${WORKDIR}/start_sdsprpcd -D ${D}${sysconfdir}/init.d/sdsprpcd

    # Install systemd unit files
    install -m 0644 ${WORKDIR}/sdsprpcd.service ${D}${systemd_unitdir}/system
}

PACKAGES_append_apq8096 = " ${PN}-sdsp"

FILES_${PN}-sdsp-dbg  = "${libdir}/.debug/libsdsp* ${bindir}/.debug/sdsprpcd"
FILES_${PN}-sdsp = "${sysconfdir}/init.d/sdsprpcd ${bindir}/sdsprpcd ${libdir}/libsdsp*.so ${libdir}/libsdsp*.so.*"

FILES_${PN}-dbg  = "${libdir}/.debug/* ${bindir}/.debug/*"
FILES_${PN}      = "${libdir}/libadsp*.so ${libdir}/libadsp*.so.* ${bindir}/adsprpcd"
FILES_${PN}     += "${libdir}/libmdsp*.so ${libdir}/libmdsp*.so.* ${bindir}/mdsprpcd"
FILES_${PN}     += "${sysconfdir}/init.d/adsprpcd ${sysconfdir}/init.d/mdsprpcd ${libdir}/pkgconfig/"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"
FILES_${PN} += "/etc/*"
