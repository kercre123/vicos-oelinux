inherit autotools qcommon qlicense
inherit autotools pkgconfig

DESCRIPTION = "libgbm Library"
PR = "r2"

SRC_DIR = "${WORKSPACE}/display/libgbm"
COLOR_METADATA_DIR = "${WORKSPACE}/display/display-hal"
S = "${WORKDIR}/libgbm/"

DEPENDS += "virtual/kernel"

EXTRA_OECONF += " --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
INSANE_SKIP_gbm += "dev-deps"

do_install_append () {
  install -d ${D}/data/display/libgbm
  install -d                                               ${D}${includedir}
  cp -rf ${S}/inc/gbm.h                                    ${D}${includedir}
  cp -rf ${S}/inc/gbm_priv.h                               ${D}${includedir}
  cp -rf ${COLOR_METADATA_DIR}/include/color_metadata.h    ${D}${includedir}
  install -d                                               ${D}${libdir}/
  cp -rf ${WORKDIR}/image/usr/lib/libgbm.so                ${D}${libdir}/
  install -d                                               ${D}${libdir}/pkgconfig
  cp -rf ${WORKDIR}/libgbm/gbm.pc                          ${D}${libdir}/pkgconfig
}

FILES_${PN} +="/data/* "
