DESCRIPTION = "QTI crypto experiments"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI = "\
file://user-data-locker.c \
file://keymaster_commands.c \
file://keymaster_commands.h \
file://keymaster_common.h \
file://keymaster_qcom.h \
file://Makefile \
file://QSEEComAPI.c \
file://QSEEComAPI.h \
file://user-data-locker.c \
"

do_compile () {
  make -C ${WORKDIR}
#  ${CC} ${WORKDIR}/user-data-locker.c -o ${WORKDIR}/user-data-locker -ldl
}

do_install() {
  echo "WORKDIR=${WORKDIR}"
  install -d ${D}/bin
  install -m 0755 ${WORKDIR}/user-data-locker ${D}/bin/
}

FILES_${PN} += "/bin"
