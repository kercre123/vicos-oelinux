inherit autotools pkgconfig qcommon qlicense qprebuilt

DESCRIPTION = "Garden Test Interfaces"
PR = "r3"

EXTRA_OECONF = "--with-core-includes=${STAGING_INCDIR}"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps/garden-app/garden-test-interfaces/"
S = "${WORKDIR}/gps/garden-app/garden-test-interfaces"
FILES_${PN} += "/usr/*"
PACKAGES = "${PN}"
