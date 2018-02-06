inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "DR core"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship/dr_amt/qdr_core/"
S = "${WORKDIR}/gps-noship/dr_amt/qdr_core"
DEPENDS = "glib-2.0 qmi-framework lbs-core loc-hal loc-pla"
EXTRA_OECONF = "--with-glib \
                --enable-a6dof=yes \
               "
do_install_append () {
        if [ -d "${SRC_DIR}" ]; then
             install -d ${D}${includedir}
             install -m 644 ${S}/qdr_core_if.h ${D}${includedir}
        fi
}

