inherit systemd
DESCRIPTION = "Animation process on failure error code service"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"

SRC_URI = "file://anki/rampost/ file://animfail.service"

TARGET_CFLAGS += "-Os -Wall -Werror -Wno-unused-result -Wno-strict-aliasing -fPIC"


S = "${WORKDIR}/anki/rampost"

do_install() {
  install -d ${D}/bin
  install -m 0755 ${S}/animfail ${D}/bin/

  install -d  ${D}${systemd_unitdir}/system/
  install -m 0644 ${WORKDIR}/animfail.service -D ${D}${systemd_unitdir}/system/animfail.service
}

FILES_${PN} += "/bin/animfail"
FILES_${PN} += "/lib/systemd/system"
