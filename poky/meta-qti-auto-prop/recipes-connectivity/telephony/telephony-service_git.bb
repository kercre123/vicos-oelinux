inherit qcommon qlicense cmake pythonnative update-rc.d

SUMMARY = "Telephony service for QTI's Modem"
DESCRIPTION = "Telephony service for QTI's Modem"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://telephony/services"
SRC_URI  += "file://services/CMakeLists.txt"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/services"

EXTRA_OECMAKE = "-DRIL_FOR_MDM_LE=ON"

INITSCRIPT_NAME = "rild"
INITSCRIPT_PARAMS = "start 90 5 3 2 . stop 10 0 1 6 ."

DEPENDS += "glib-2.0 nanopb system-core native-frameworks"
DEPENDS += "data diag qmi qmi-framework qmi-client-helper"
DEPENDS += "sqlite3 libxml2 "

INSANE_SKIP_${PN} += "dev-deps"

RDEPENDS_${PN} += "liblog libcutils"
