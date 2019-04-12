inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS lowi client"
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps-noship/internal/lowi/"
FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://gps-noship/internal/lowi/"
S = "${WORKDIR}/gps-noship/internal/lowi/"
DEPENDS = "glib-2.0 loc-base-util loc-mq-client loc-pla"
EXTRA_OECONF = "--with-glib"
do_install_append() {
if [ -d "${SRC_DIR}" ]; then
   install -d ${D}/usr/include
   install -d ${D}/usr/include/lowi
   install -d ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_client.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_client_receiver.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_const.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_defines.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_mac_address.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_request.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_response.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_scan_measurement.h ${D}/usr/include/lowi/inc
   install -m 0644 ${S}/inc/lowi_ssid.h ${D}/usr/include/lowi/inc
else
   qprebuilt_do_install
fi
}
