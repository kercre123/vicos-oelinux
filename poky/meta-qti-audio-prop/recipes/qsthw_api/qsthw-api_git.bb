inherit autotools qcommon qlicense

DESCRIPTION = "qsthw_api Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio/qsthw_api/"
S = "${WORKDIR}/audio/mm-audio/qsthw_api/"

DEPENDS = "tinyalsa libcutils libhardware soundtrigger"

EXTRA_OECONF += "BOARD_SUPPORTS_QSTHW_API=true"
EXTRA_OEMAKE = "DEFAULT_INCLUDES="-I${STAGING_INCDIR}/sound_trigger""

SOLIBS = ".so"
FILES_SOLIBSDEV = ""
