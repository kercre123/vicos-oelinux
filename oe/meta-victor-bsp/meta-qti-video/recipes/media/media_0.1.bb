inherit autotools pkgconfig sdllvm

DESCRIPTION = "media"

LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/${LICENSE};md5=3775480a712fc46a69647678acb234cb"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://hardware/qcom/media/"
SRC_URI  += "file://msm8953-version.sh"

S = "${WORKDIR}/hardware/qcom/media"

DEPENDS = "adreno200"
DEPENDS += "display-hal"
DEPENDS += "system-media"
DEPENDS += "av-frameworks"
DEPENDS += "glib-2.0"
DEPENDS += "mm-video-noship"

# 8017 need display-hal-linux to compile
DEPENDS_append_apq8017 += "display-hal-linux"

# 8909 don't need dispaly-ha to compile.
DEPENDS_remove_msm8909 = "display-hal mm-video-noship"
DEPENDS_remove_apq8017 = "display-hal mm-video-noship"

# configure features
EXTRA_OECONF_append =" --enable-use-glib="yes""
EXTRA_OECONF_append =" --enable-target-uses-ion="yes""
EXTRA_OECONF_append =" --enable-target-${SOC_FAMILY}="yes""
EXTRA_OECONF_append =" --enable-target-uses-media-extensions="no""
EXTRA_OECONF_append_msm8909 =" --enable-targets-that-use-flag-msm8226="yes""
EXTRA_OECONF_append_msm8916 =" --enable-targets-that-use-flag-msm8226="yes""
EXTRA_OECONF_append_msm8953 =" --enable-build-mm-video="yes""
EXTRA_OECONF_append_apq8017 =" --enable-build-mm-video="yes""
EXTRA_OECONF_append_msm8953 =" --enable-targets-that-support-pq="yes""
EXTRA_OECONF_append_msm8953 =" --enable-targets-that-support-vqzip="yes""
EXTRA_OECONF_append_msm8953 =" --enable-targets-that-support-adsp-pq="yes""
EXTRA_OECONF_append_msm8996 =" --enable-build-mm-video="yes""
EXTRA_OECONF_append_msm8996 =" --enable-is-ubwc-supported="yes""
EXTRA_OECONF_append_msm8996 =" --enable-targets-that-support-pq="yes""
EXTRA_OECONF_append_msm8996 =" --enable-master-side-cp-target-list="yes""
EXTRA_OECONF_append_msm8996 =" --enable-targets-that-support-vqzip="yes""
EXTRA_OECONF_append_msm8998 =" --enable-targets-that-support-vqzip="yes""
EXTRA_OECONF_append_msmcobalt =" --enable-is-ubwc-supported="yes""

# configure headers
EXTRA_OECONF_append =" --with-glib"
EXTRA_OECONF_append =" --with-ui-headers=${STAGING_INCDIR}/ui/"
EXTRA_OECONF_append =" --with-android-headers=${STAGING_INCDIR}/"
#EXTRA_OECONF_append =" --with-utils-headers=${STAGING_INCDIR}/utils/"
#EXTRA_OECONF_append =" --with-cutils-headers=${STAGING_INCDIR}/cutils/"
EXTRA_OECONF_append =" --with-glib-headers=${STAGING_INCDIR}/glib-2.0/"
EXTRA_OECONF_append =" --with-binder-headers=${STAGING_INCDIR}/binder/"
EXTRA_OECONF_append =" --with-adreno-headers=${STAGING_INCDIR}/adreno/"
EXTRA_OECONF_append =" --with-glib-lib-dir=${STAGING_LIBDIR}/glib-2.0/include"
EXTRA_OECONF_append =" --with-gralloc-headers=${STAGING_INCDIR}/libgralloc/"
EXTRA_OECONF_append =" --with-qdutils-headers=${STAGING_INCDIR}/libqdutils/"
EXTRA_OECONF_append =" --with-libgpustats-headers=${STAGING_INCDIR}/libgpustats/"
EXTRA_OECONF_append =" --with-libpqstats-headers=${STAGING_INCDIR}/libpqstats/"
EXTRA_OECONF_append =" --with-libvqzipstats-headers=${STAGING_INCDIR}/libvqzip/"
EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
EXTRA_OECONF_append =" --with-display-headers=${STAGING_INCDIR}/qcom/display"

FILES_${PN}-dbg  = "${libdir}/.debug/*"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${libdir}/*.so.*.*.* ${sysconfdir}/* ${bindir}/* ${libdir}/pkgconfig/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"

do_install_append() {
    oe_runmake DESTDIR="${D}/" LIBVER="${LV}" install
    mkdir -p ${STAGING_INCDIR}/mm-core
    mkdir -p ${STAGING_INCDIR}/libstagefrighthw
    install -m 0644 ${S}/mm-core/inc/*.h ${STAGING_INCDIR}/mm-core
    install -m 0644 ${S}/libstagefrighthw/*.h ${STAGING_INCDIR}/libstagefrighthw
}

do_install_append_apq8053() {
    install -m 0755 ${WORKDIR}/msm8953-version.sh -D ${D}${sysconfdir}/init.d/msm8953-version.sh
}

pkg_postinst_${PN}_apq8053 () {
    [ -n "$D" ] && OPT="-r $D" || OPT="-s"
    # remove all rc.d-links potentially created from alternatives
    update-rc.d $OPT -f msm8953-version.sh remove
    update-rc.d $OPT msm8953-version.sh start 60 2 3 4 5 .
}

INSANE_SKIP_${PN} += "dev-so"
EXCLUDE_FROM_SHLIBS = "1"
