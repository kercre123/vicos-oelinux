DESCRIPTION = "Include LA system libs "
LICENSE = "BSD-3-Clause"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/BSD-3-Clause;md5=550794465ba0ec5312d6919e203a55f9"

SRC_URI += "file://lib*.so"
SRC_URI += "file://linker"
SCR_URI += "file://display"

do_install_append() {
  install -d ${D}/system/bin
  install -m 755 ${WORKDIR}/linker ${D}/system/bin
  install -m 755 ${WORKDIR}/display ${D}/system/bin
  install -d ${D}/system/lib
  install -m 755 ${WORKDIR}/lib*.so ${D}/system/lib
  install -m 644 ${WORKDIR}/*.o ${D}/system/lib
}

do_package_qa() {
  bbwarn "LA libs should be removed as soon as we are using proper build tools ~daniel@anki.com"
}

FILES_${PN} += "/system"
