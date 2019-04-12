inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "LBS Core hdr"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps/framework/native/core/"
S = "${WORKDIR}/gps/framework/native/core"
FILES_${PN} += "/usr/*"
PACKAGES = "${PN}"

do_configure() {
}

do_compile() {
}

do_install() {
    install -d ${D}${includedir}
    install -m 644 ${S}/*.h ${D}${includedir}
}
