inherit autotools qcommon pkgconfig qlicense qprebuilt sdllvm

SUMMARY = "mm parser noship"
SECTION = "multimedia"

DESCRIPTION = "mm-parser-noship"

PR = "r4"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://video/lib/mm-parser-noship/"

SRC_DIR = "${WORKSPACE}/video/lib/mm-parser-noship"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/video/lib/mm-parser-noship"

#for common inc header
DEPENDS += "glib-2.0"
DEPENDS += "mm-osal"
DEPENDS += "liblog"
DEPENDS += "libcutils"

EXTRA_OECONF_append =" --with-cutils-headers=${STAGING_INCDIR}/cutils/"
EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/"
EXTRA_OECONF_append =" --with-mmparser-headers=${WORKSPACE}/video/lib/mm-parser/Api/inc/"
EXTRA_OECONF_append =" --with-mmosal-headers=${STAGING_INCDIR}/mm-osal/include/"
EXTRA_OECONF_append =" --with-glib"


PACKAGES = "${PN}-dbg ${PN} ${PN}-dev"

FILES_${PN}-dbg = "${libdir}/.debug/*"
FILES_${PN}     = "${libdir}/lib*.so.* ${libdir}/lib*.so"
FILES_${PN}-dev = "${libdir}/lib*.so ${libdir}/*.la ${includedir}"

do_install_append() {
  install -d ${D}${includedir}/mm-parser/include/
  install -m 0644 ${S}/FileBaseLib/inc/isucceedfail.h -D ${D}${includedir}/mm-parser/include/isucceedfail.h
  install -m 0644 ${S}/FileBaseLib/inc/parserinternaldefs.h -D ${D}${includedir}/mm-parser/include/parserinternaldefs.h
  install -m 0644 ${S}/FileBaseLib/inc/oscl_file_io.h -D ${D}${includedir}/mm-parser/include/oscl_file_io.h
  install -m 0644 ${S}/FileBaseLib/inc/filesourcestring.h -D ${D}${includedir}/mm-parser/include/filesourcestring.h
  install -m 0644 ${S}/FileBaseLib/inc/zrex_string.h -D ${D}${includedir}/mm-parser/include/zrex_string.h
}
INSANE_SKIP_${PN} += "dev-so"
EXCLUDE_FROM_SHLIBS = "1"
