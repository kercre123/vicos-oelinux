require ${COREBASE}/meta-qti-qsap/recipes/gstreamer/gstreamer1.0-plugins.inc
require ${COREBASE}/meta-qti-qsap/recipes/gstreamer/gstreamer-common-submodule.inc

LICENSE = "Qualcomm-Technologies-Inc.-Proprietary"

DEPENDS += ""

inherit gettext

PACKAGES_DYNAMIC =+ "^libgst.*"

PACKAGECONFIG ??= " \
    "

EXTRA_OECONF += " \
"

CACHED_CONFIGUREVARS_append_x86 = " ac_cv_header_emmintrin_h=no ac_cv_header_xmmintrin_h=no"
