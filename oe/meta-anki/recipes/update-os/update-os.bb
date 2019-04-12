DESCRIPTION = "Anki developer script to update OS on robot"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"


SRC_URI += "file://update-os.sh"

do_install_append() {
   install -d ${D}/sbin
   install -m 0700 ${WORKDIR}/update-os.sh ${D}/sbin/update-os
}


