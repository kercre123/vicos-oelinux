inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Izat Core"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://gps-noship/framework/native/core/"
SRC_DIR = "${WORKSPACE}/gps-noship/framework/native/core/"
S = "${WORKDIR}/gps-noship/framework/native/core"
DEPENDS = "glib-2.0 qmi-framework loc-pla loc-hal lbs-core-hdr openssl sqlite3 system-core gps-utils loc-net-iface loc-base-util"
EXTRA_OECONF ="--with-glib \
               --with-lbscore-includes=${STAGING_INCDIR}/lbs-core-hdr"

PACKAGES = "${PN}"
INHIBIT_PACKAGE_DEBUG_SPLIT = "1"
FILES_${PN} = "${libdir}/*"
FILES_${PN} += "/usr/include/*"
FILES_${PN} += "/etc/*"

# The izat-core package contains symlinks that trip up insane
INSANE_SKIP_${PN} = "dev-so"
