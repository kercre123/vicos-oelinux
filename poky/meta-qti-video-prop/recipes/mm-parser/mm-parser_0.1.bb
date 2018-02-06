inherit autotools deploy qlicense qprebuilt pkgconfig sdllvm

SUMMARY = "mm parser"
SECTION = "multimedia"

PR = "r4"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://video/lib/mm-parser/"

SRC_DIR = "${WORKSPACE}/video/lib/mm-parser"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/video/lib/mm-parser"

DEPENDS = "glib-2.0"
DEPENDS += "mm-osal"

EXTRA_OECONF_append =" --with-mmosal-headers=${STAGING_INCDIR}/mm-osal/include/"

PACKAGES = "${PN}-dbg ${PN} ${PN}-dev"

#INSANE_SKIP_${PN} = "installed-vs-shipped"

FILES_${PN}-dbg = "${libdir}/.debug/*"
FILES_${PN}     = "${includedir}/*"
FILES_${PN}-dev = "${bindir}/* ${includedir}"

do_install_append() {
   install -d ${D}${includedir}/mm-parser/include/
   install -m 0644 ${S}/Api/inc/*.h -D ${D}${includedir}/mm-parser/include/
}
