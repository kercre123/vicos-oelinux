inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "Bluetooth BT Vendor Filters"
PR = "r0"
DEPENDS = "common qmi qmi-framework glib-2.0"

RDEPENDS_${PN} = "libcutils"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://vendor/qcom/proprietary/bt/hci_qcomm_init/"
SRC_DIR = "${WORKSPACE}/vendor/qcom/proprietary/bt/hci_qcomm_init/"
S = "${WORKDIR}/vendor/qcom/proprietary/bt/hci_qcomm_init/"

BASEPRODUCT = "${@d.getVar('PRODUCT', False)}"

EXTRA_OECONF = "--with-lib-path=${STAGING_LIBDIR} \
                --with-glib \
                --enable-target=${BASEMACHINE} \
                --enable-rome=${BASEPRODUCT} \
               "

CFLAGS_append = " -DUSE_ANDROID_LOGGING "
LDFLAGS_append = " -llog "

do_install_append () {

    if [ -d "${SRC_DIR}" ]; then
        install -d ${D}${includedir}
        install -m 644 ${S}/bt_nv.h ${D}${includedir}
        install -m 644 ${S}/btqsocnvm.h ${D}${includedir}
        install -m 644 ${S}/btqsocnvmutils.h ${D}${includedir}
    fi
}
