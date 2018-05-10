inherit systemd
DESCRIPTION = "Anki Robot Early Boot Self Test"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SRC_URI = "file://rampost.service \
           file://rampost.c \
           file://spine_hal.c \
           file://gpio.c \
           file://lcd.c \
           file://imu.c \
           file://rampost.h \
           file://spine_hal.h \
           file://gpio.h \
           file://lcd.h \
           file://messages.h \
           file://animfail.c \
           file://error_565.h \
           file://warning_orange.h \
           file://animfail.service \
           "

do_compile () {
  ${CC} ${WORKDIR}/rampost.c ${WORKDIR}/gpio.c ${WORKDIR}/lcd.c ${WORKDIR}/spine_hal.c -o ${WORKDIR}/rampost
  ${CC} ${WORKDIR}/animfail.c ${WORKDIR}/gpio.c ${WORKDIR}/lcd.c -o ${WORKDIR}/animfail
}

do_install() {
  install -d ${D}/bin
  install -m 0755 ${WORKDIR}/rampost ${D}/bin/
  install -m 0755 ${WORKDIR}/animfail ${D}/bin/

  install -d  ${D}${systemd_unitdir}/system/
  install -m 0644 ${WORKDIR}/animfail.service -D ${D}${systemd_unitdir}/system/animfail.service
  install -m 0644 ${WORKDIR}/rampost.service -D ${D}${systemd_unitdir}/system/rampost.service
  install -d ${D}${systemd_unitdir}/system/sysinit.target.wants/
  # enable the service for sysinit.target
  ln -sf ${systemd_unitdir}/system/rampost.service \
       ${D}${systemd_unitdir}/system/sysinit.target.wants/rampost.service
}

FILES_${PN} += "/bin"
FILES_${PN} += "/lib/systemd/system"
