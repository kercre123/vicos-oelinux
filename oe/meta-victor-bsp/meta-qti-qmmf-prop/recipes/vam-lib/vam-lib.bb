inherit autotools pkgconfig qlicense

DESCRIPTION = "VAM"

PR = "r0"

DEPENDS += "system-core "

FILESPATH =+ "${WORKSPACE}/vendor/qcom/proprietary/:"
SRC_URI   = "file://vam"

S = "${WORKDIR}/vam/vam_lib"

do_install_append() {

       install -d ${D}${includedir}/VAM
       install -m 0644  ${S}/inc/* ${D}${includedir}/VAM

}

FILES_${PN} += "/usr"

PACKAGES =+ "${PN}-libvamanager-dbg ${PN}-libvamanager ${PN}-libvamanager-dev"
FILES_${PN}-libvamanager-dbg    = "${libdir}/VAM/.debug/libVAManager.*"
FILES_${PN}-libvamanager        = "${libdir}/VAM/libVAManager.so.*"
FILES_${PN}-libvamanager-dev    = "${libdir}/VAM/libVAManager.so ${libdir}/VAM/libVAManager.la ${includedir}"





