inherit qcommon qprebuilt qlicense

DESCRIPTION = "ffv Library"
PR = "r0"

SRC_DIR = "${WORKSPACE}/audio/mm-audio-noship/ffv/"
S = "${WORKDIR}/audio/mm-audio-noship/ffv/"

DEPENDS = "virtual/kernel"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""
