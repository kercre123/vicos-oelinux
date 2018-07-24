inherit systemd
DESCRIPTION = "Early animation process"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"

SRC_URI = "\
file://early-anim.service \
file://early-anim \
"

S = "${WORKDIR}"

do_install() {
  install -d ${D}/bin
  install -m 0755 ${S}/early-anim ${D}/bin/

  install -d  ${D}${systemd_unitdir}/system/
  install -m 0644 ${WORKDIR}/early-anim.service -D ${D}${systemd_unitdir}/system/early-anim.service
}

FILES_${PN} += "/bin/early-anim"
FILES_${PN} += "/lib/systemd/system"
