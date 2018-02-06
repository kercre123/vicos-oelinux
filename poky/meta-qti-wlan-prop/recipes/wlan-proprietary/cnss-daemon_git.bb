inherit autotools qcommon qlicense update-rc.d qprebuilt

DESCRIPTION = "CNSS"
PR = "r2"

DEPENDS = "qmi data libcutils libnl"

RDEPENDS_${PN} = "data"

FILESPATH =+ "${WORKSPACE}/wlan-proprietary/:"

SRC_URI = "file://cnss-daemon/"

SRC_DIR = "${WORKSPACE}/wlan-proprietary/cnss-daemon"

S = "${WORKDIR}/cnss-daemon"

CFLAGS += "-I ${STAGING_INCDIR}/libnl3"
CFLAGS += "-I ${WORKSPACE}/system/core/include/"

EXTRA_OECONF = "--enable-debug"

INITSCRIPT_NAME = "start_cnss_daemon"
INITSCRIPT_PARAMS = "start 90 2 3 4 5 . stop 10 0 1 6 ."
