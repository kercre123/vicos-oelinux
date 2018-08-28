DESCRIPTION = "User Data Locker"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://anki/user-data-locker/"

S = "${WORKDIR}/anki/user-data-locker/"

do_install() {
  mkdir -p ${D}/bin
  install -m 0100 ${S}/user-data-locker ${D}/bin/
}
