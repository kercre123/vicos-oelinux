inherit autotools qcommon
inherit qlicense qprebuilt


SUMMARY = "mm-video-noship"
SECTION = "multimedia"
LICENSE = "Qualcomm-Technologies-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp-prop/files/qcom-licenses/\
Qualcomm-Technologies-Inc.-Proprietary;md5=92b1d0ceea78229551577d4284669bb8"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://video/lib/mm-video-noship/"
#SRC_URI += "file://0001-enable-gpu-stats.patch"
SRC_URI += "file://fpv.cfg"
SRC_DIR = "${WORKSPACE}/video/lib/mm-video-noship"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/video/lib/mm-video-noship"

PACKAGES = "${PN}"

DEPENDS += "libnl"
DEPENDS += "display-hal"
DEPENDS += "adreno200"
DEPENDS += "adsprpc"
DEPENDS += "glib-2.0"

EXTRA_OEMAKE += "LINUX_OMX_TEST_ONLY_ENCODE=true"
EXTRA_OEMAKE += "LOCAL_CLANG=false"
EXTRA_OEMAKE += "LINUX_FPV_RAVE_ENABLED=true"

FILES_${PN} += "${sysconfdir}/fpv.cfg \
                ${includedir}/fpv_rave/fpv_config.hpp \
                ${includedir}/fpv_rave/fpv_dbg.hpp \
                ${includedir}/fpv_rave/fpv_queue.hpp \
                ${includedir}/fpv_rave/fpv_ra.hpp \
                ${includedir}/fpv_rave/fpv_rave.hpp \
                ${includedir}/fpv_rave/fpv_utils.hpp"


EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
EXTRA_OECONF_append =" --with-kernel-headers=${STAGING_KERNEL_BUILDDIR}/include"
EXTRA_OECONF_append =" --with-uapi-headers=${STAGING_KERNEL_DIR}/include/uapi/media/"
EXTRA_OECONF_append =" --with-xml-includes=${STAGING_INCDIR}/libxml2"
EXTRA_OECONF_append =" --with-media-includes=${STAGING_KERNEL_DIR}/include/media/"
EXTRA_OECONF_append =" --with-netlink-includes=${STAGING_INCDIR}/libnl3/"
EXTRA_OECONF_append =" --with-adreno-includes=${STAGING_INCDIR}/adreno"
EXTRA_OECONF_append =" --with-utils-headers=${STAGING_INCDIR}/utils/"
EXTRA_OECONF_append =" --with-cutils-headers=${STAGING_INCDIR}/cutils/"
EXTRA_OECONF_append =" --with-log-headers=${STAGING_INCDIR}/log/"
EXTRA_OECONF_append =" --with-videolibs-headers=${WORKDIR}/video/lib/mm-video-noship/utils/inc"
EXTRA_OECONF_append =" --with-libyuvtool-headers=${STAGING_INCDIR}/libyuvtool/"
EXTRA_OECONF_append =" --with-usr-include-headers=${STAGING_INCDIR}/"

EXTRA_OECONF_append_msm8909   =" --enable-is-swvenc-enable="yes""
EXTRA_OECONF_append_msm8937   =" --enable-is-swvenc-enable="yes""
EXTRA_OECONF_append_msm8937   =" --enable-is-swvdec-enable="yes""
EXTRA_OECONF_append_msm8974   =" --enable-is-hevc-enable="yes""
EXTRA_OECONF_append_msm8226   =" --enable-is-hevc-enable="yes""
EXTRA_OECONF_append_msm8916   =" --enable-is-hevc-enable="yes""
EXTRA_OECONF_append_msm8998   =" --enable-is-gpupq-enable="yes""
EXTRA_OECONF_append_msm8996   =" --enable-is-gpupq-enable="yes""
EXTRA_OECONF_append_sdm660    =" --enable-is-gpupq-enable="yes""
EXTRA_OECONF_append_msm8953   =" --enable-is-gpupq-enable="yes""
EXTRA_OECONF_append_msm8992   =" --enable-is-hevc-enc-enable="yes""
EXTRA_OECONF_append_msm8909   =" --enable-is-ittiam-vidc-enable="yes""
EXTRA_OECONF_append_msm8953   =" --enable-is-adsppq-enable="yes""
EXTRA_OECONF_append_msm8996   =" --enable-is-fpv-enable="yes""
EXTRA_OECONF_append_msm8953   =" --enable-is-fpv-enable="yes""
EXTRA_OECONF_append_msm8998   =" --enable-is-vzip-enable="yes""
EXTRA_OECONF_append_msm8996   =" --enable-is-vzip-enable="yes""
EXTRA_OECONF_append_msm8953   =" --enable-is-vzip-enable="yes""
EXTRA_OECONF_append_thulium   =" --enable-is-ubwc-supported="yes""
EXTRA_OECONF_append_msm8996   =" --enable-is-ubwc-supported="yes""
EXTRA_OECONF_append_sdm660    =" --enable-is-ubwc-supported="yes""
EXTRA_OECONF_append_msm8974   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8226   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_apq8084   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8916   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8994   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8909   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8992   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8996   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8952   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8937   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msm8953   =" --enable-is-venus_target="yes""
EXTRA_OECONF_append_msmcobalt =" --enable-is-venus_target="yes""

FILES_${PN}-dbg  = "${libdir}/.debug/*"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${libdir}/pkgconfig/* ${bindir}/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"
INSANE_SKIP_${PN} = "dev-so"

do_install_append() {
install -d ${D}${includedir}/mm-video/utils
install -m 0644 ${S}/utils/inc/list.h -D ${D}${includedir}/mm-video/utils/
install -m 0644 ${S}/utils/inc/VideoComDef.h -D ${D}${includedir}/mm-video/utils/VideoComDef.h

install -d ${D}${includedir}/mm-video/fastcrc
install -m 0644 ${S}/fastcrc/inc/VideoCRCChecker.h -D ${D}${includedir}/mm-video/fastcrc/VideoCRCChecker.h

install -d ${D}${includedir}/mm-video/ubwc
install -m 0644 ${S}/ubwc/inc/ubwc.h -D ${D}${includedir}/mm-video/ubwc/ubwc.h
install -m 0644 ${S}/ubwc/inc/ubwc_internal.h -D ${D}${includedir}/mm-video/ubwc/ubwc_internal.h

install -d ${D}${includedir}/mm-video/streamparser
install -m 0644 ${S}/streamparser/inc/VideoStreamParser.h -D ${D}${includedir}/mm-video/streamparser/VideoStreamParser.h

#install pqstats header file
install -d ${D}${includedir}/libpqstats
install -m 0644 ${S}/pq_stats/common/inc/pqstats.h -D ${D}${includedir}/libpqstats/pqstats.h



# install FPV config file
dest=/etc
install -d ${D}${dest}
install -m 755 ${WORKDIR}/fpv.cfg ${D}${dest}

# install FPV header file
install -d ${D}${includedir}/fpv_rave
cp -r ${S}/fpv_rave/inc/* ${D}${includedir}/fpv_rave/
}

do_install_append_apq8053(){
#install vqzip header file
install -d ${D}${includedir}/libvqzip
install -m 0644 ${S}/vqzip/VQZip.h -D ${D}${includedir}/libvqzip/VQZip.h

if [ "${MLPREFIX}" == "lib32-" ]; then
 install -d ${D}${libdir}/rfsa/adsp
 install -m 0644 ${S}/pq_stats/adsp_skel/*.so -D ${D}${libdir}/rfsa/adsp/
fi
}

do_install_append_apq8098(){
install -d ${D}${includedir}/libvqzip
install -m 0644 ${S}/vqzip/VQZip.h -D ${D}${includedir}/libvqzip/VQZip.h
}

do_install_append_apq8096(){
install -d ${D}${includedir}/libvqzip
install -m 0644 ${S}/vqzip/VQZip.h -D ${D}${includedir}/libvqzip/VQZip.h
}

INSANE_SKIP_${PN} += "arch"
export TARGET_LIBRARY_SUPPRESS_LIST="libadsprpc"
EXCLUDE_FROM_SHLIBS = "1"
