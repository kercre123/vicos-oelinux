inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Qualcomm Atheros ath6kl utils."
DEPENDS = "diag libnl glib-2.0"

PR = "r2"

SRC_DIR = "${WORKSPACE}/wlan-proprietary/ath6kl-utils"

S = "${WORKDIR}/ath6kl-utils"

EXTRA_OECONF = "--with-glib"
EXTRA_OECONF += "--enable-target=${BASEMACHINE}"
