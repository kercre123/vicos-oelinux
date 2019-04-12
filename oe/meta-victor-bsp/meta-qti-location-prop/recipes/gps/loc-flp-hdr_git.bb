inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "flp common hdr"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/flp/"
S = "${WORKDIR}/gps-noship/flp"
do_configure() {
}

do_compile() {
}
FILES_${PN} += "/usr/*"
PACKAGES = "${PN}"

do_install() {
    if [ -d "${SRC_DIR}" ]; then
	install -d ${D}${includedir}
	install -m 644 ${S}/fused_location_extended.h ${D}${includedir}
    else
	qprebuilt_do_install
    fi
}
