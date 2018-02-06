inherit autotools-brokensep pkgconfig update-rc.d qprebuilt

DESCRIPTION = "PD mapper"
LICENSE = "Qualcomm-Technologies-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp-prop/files/qcom-licenses/\
${LICENSE};md5=92b1d0ceea78229551577d4284669bb8"

PR = "r1"
PACKAGES = "${PN}"
FILESPATH =+ "${WORKSPACE}:"

SRC_URI = "file://ss-services"
SRC_DIR = "${WORKSPACE}/ss-services/"
S = "${WORKDIR}/ss-services"

DEPENDS = "glib-2.0 json-c system-core qmi qmi-framework libcutils"

CPPFLAGS += "-I${WORKSPACE}/qmi-framework/inc"
CPPFLAGS += "-I${WORKSPACE}/system-core/"

INITSCRIPT_NAME = "start_pdmappersvc"
INITSCRIPT_PARAMS = "start 90 2 3 4 5 . stop 10 0 1 6 ."

EXTRA_OEMAKE = "DEFAULT_INCLUDES= CFLAGS="-I. -I${WORKSPACE}/ss-services/pd-mapper/pd-mapper-idl/ -I${STAGING_INCDIR}/cutils/ -I${STAGING_INCDIR}/qmi/ -I${WORKDIR}/ss-services/pd-mapper/pd-mapper-svc -I${WORKDIR}/ss-services/pd-notifier/pd-notifier-idl/ -I${WORKSPACE}/ss-services/pd-mapper/libpdmapper/ -I${WORKSPACE}/ss-services/pd-notifier/libpdnotifier/ -I${STAGING_KERNEL_BUILDDIR}/usr/include""
EXTRA_OEMAKE += "DEFAULT_INCLUDES= CPPFLAGS="-I. -I${WORKSPACE}/ss-services/pd-mapper/pd-mapper-idl/ -I${STAGING_INCDIR}/qmi-framework/ -I${STAGING_INCDIR}/qmi/ -I${STAGING_INCDIR}/json-c/ -I${STAGING_INCDIR}/cutils/ -I${WORKSPACE}/pd-notifier/pd-notifier-idl/ -I${WORKSPACE}/ss-services/pd-mapper/libpdmapper/ -I${WORKSPACE}/ss-services/pd-notifier/libpdnotifier/ -I${WORKSPACE}/system/core/include/""

do_install_append() {
       install -m 0755 ${S}/pd-mapper/start_pdmappersvc -D ${D}${sysconfdir}/init.d/start_pdmappersvc
}

