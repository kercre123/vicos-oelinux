inherit qcommon qlicense autotools pkgconfig update-rc.d

PACKAGES = "${PN}-dbg ${PN} ${PN}-doc ${PN}-dev ${PN}-staticdev ${PN}-locale"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://telephony/lib/"
SRC_URI += "file://start_rilmuxd"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/telephony/lib"

FILES_${PN} = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${libdir}/pkgconfig/*"
FILES_${PN} += "/data/tel.conf"
FILES_${PN} += "/usr/bin/*"
FILES_${PN} += "/etc/init.d/*"


DEPENDS += "native-frameworks libcutils telephony-service qmi qmi-framework"

CFLAGS += " -pthread -fPIC --std=c++14"
LDFLAGS += " -lbinder -lcutils -llog -lutils"

INITSCRIPT_NAME = "rilmuxd"
INITSCRIPT_PARAMS = "start 91 5 3 2 . stop 10 0 1 6 ."

do_install_append() {
    install -m 0644 ${S}/config/tel.conf -D ${D}/data/tel.conf
    install -m 0755 ${WORKDIR}/start_rilmuxd -D ${D}${sysconfdir}/init.d/${INITSCRIPT_NAME}
}
INSANE_SKIP_${PN} += "dev-deps"
