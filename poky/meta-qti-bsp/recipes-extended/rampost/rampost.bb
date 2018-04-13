DESCRIPTION = "Anki Robot RAMPOST Utility"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"


SRC_URI = "file://rampost.c"

do_compile () {
  ${CC} ${WORKDIR}/rampost.c -o ${WORKDIR}/rampost
}

do_install() {
  install -d ${D}/bin
  install -m 0755 ${WORKDIR}/rampost ${D}/bin/
}

FILES_${PN} += "/bin/rampost"
