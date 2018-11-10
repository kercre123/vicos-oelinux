DESCRIPTION = "Anki sudo config"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESEXTRAPATHS_prepend := "${THISDIR}/files:"

SRC_URI += "file://sudoersd-anki"

do_install_append () {
   install -m 0750 -d ${D}${sysconfdir}/sudoers.d
   install -m 0644 ${WORKDIR}/sudoersd-anki -D ${D}${sysconfdir}/sudoers.d/anki
}

FILES_${PN} += "${sysconfdir}/sudoers.d/anki"
