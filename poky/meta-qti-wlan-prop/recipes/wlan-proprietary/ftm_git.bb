inherit autotools qcommon qprebuilt qlicense

DESCRIPTION = "Qualcomm Technologies ftmdaemon"
DEPENDS = "libnl diag glib-2.0 system-core libbt-vendor hci-qcomm-init ath6kl-utils"

PR = "r1"

SRC_DIR = "${WORKSPACE}/wlan-proprietary/ftm/"

S = "${WORKDIR}/wlan-proprietary/ftm"

BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"

EXTRA_OECONF = " \
                --with-glib \
                --enable-wlan=yes \
                --enable-bt=yes \
                --enable-debug=yes \
                --enable-target=${BASEMACHINE} \
                --enable-rome=${BASEPRODUCT} \
                "
