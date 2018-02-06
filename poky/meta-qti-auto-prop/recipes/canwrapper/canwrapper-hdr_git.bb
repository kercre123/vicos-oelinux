inherit qcommon qlicense qprebuilt

DESCRIPTION = "CAN Wrapper hdr"
PR = "r1"

SRC_DIR = "${WORKSPACE}/vnw/"
S = "${WORKDIR}/vnw"

do_configure() {
}

do_compile() {
}

do_install() {
    install -d ${D}${includedir}
    install -m 644 ${S}/CanWrapper/*.h ${D}${includedir}
}
