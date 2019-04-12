inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS xtwifi tile"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/xtwifi/"
S = "${WORKDIR}/gps-noship/xtwifi"
DEPENDS = "glib-2.0 izat-core gdtap-adapter loc-base-util loc-launcher lowi-client sqlite3 asn1c-cper asn1c-crt asn1c-rtx loc-pla"
EXTRA_OECONF = "--with-glib"

PACKAGES = "${PN}"
FILES_${PN} += "${libdir}/*.so"
FILES_${PN} += "/usr/include/*"
FILES_${PN} += "/usr/lib/*"
FILES_${PN} += "/usr/bin/*"
INSANE_SKIP_${PN} = "dev-so"
