inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "power manager"
PR = "r1"

SRC_DIR = "${WORKSPACE}/vnw/powermanagerdaemon/lib/"
DEPENDS = "glib-2.0 qmi-framework"
EXTRA_OECONF ="--with-glib"

S = "${WORKDIR}/vnw/powermanagerdaemon/lib/"

do_install_append(){
    install -d ${D}${includedir}
    install -m 0555 ${S}/power_state.h ${D}${includedir}
}
