DESCRIPTION = "QTI crypto experiments"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI = " file://user-data-locker.c \
           "

#DEPENDS += "libcryptofs"

do_compile () {
#  ${CC} ${WORKDIR}/user-data-locker.c -o ${WORKDIR}/user-data-locker -lcryptfs_hw
  ${CC} ${WORKDIR}/user-data-locker.c -o ${WORKDIR}/user-data-locker -ldl
}

do_install() {
  install -d ${D}/bin
  install -m 0755 ${WORKDIR}/user-data-locker ${D}/bin/
}

FILES_${PN} += "/bin"
