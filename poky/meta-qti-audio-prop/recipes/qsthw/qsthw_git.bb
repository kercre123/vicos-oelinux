inherit autotools qcommon qlicense

DESCRIPTION = "qsthw Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio/qsthw/"
S = "${WORKDIR}/audio/mm-audio/qsthw/"

DEPENDS = "tinyalsa libcutils libhardware soundtrigger"

EXTRA_OECONF += "BOARD_SUPPORTS_QSTHW_API=true"
EXTRA_OECONF_append_apq8009 = " ENABLE_KEEP_ALIVE=true"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""
