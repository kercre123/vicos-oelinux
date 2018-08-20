DESCRIPTION = "User Data Locker"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI = "\
file://user-data-locker.c \
file://Makefile \
"

do_compile () {
  make -C ${WORKDIR}
}

do_install() {
  echo "WORKDIR=${WORKDIR}"
  install -d ${D}/bin
  install -m 0755 ${WORKDIR}/user-data-locker ${D}/bin/
}

FILES_${PN} += "/bin"
