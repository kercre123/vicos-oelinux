inherit qcommon qlicense qprebuilt systemd

DESCRIPTION = "Data Path Optimizer to achieve higher throughputs"
LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

PR = "r2"

DEPENDS = "configdb dsutils"

S = "${WORKDIR}/data-noship/data_path_opt/"
SRC_DIR = "${WORKSPACE}/data-noship/data_path_opt/"

EXTRA_OECONF = "--with-common-includes=${STAGING_INCDIR}"



