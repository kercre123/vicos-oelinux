DESCRIPTION = "Anki Version Number"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"                                                                        
SRC_URI = "file://ANKI_VERSION"

do_install_append () {
   install -d ${D}/etc
   install -m 0444 ${WORKDIR}/ANKI_VERSION ${D}/etc/anki-version-base

   # ANKI_BUILD_VERSION will be set by TeamCity for CI builds, default to 0 for developer builds    
   : ${ANKI_BUILD_VERSION:=0}
   echo "${ANKI_BUILD_VERSION}" > ${D}/etc/anki-version-code
   chmod 0444 ${D}/etc/anki-version-code

   BASE_VERSION=$(cat ${WORKDIR}/ANKI_VERSION)
   echo "${BASE_VERSION}.${ANKI_BUILD_VERSION}" > ${D}/etc/anki-version
   chmod 0444 ${D}/etc/anki-version
}

FILES_${PN} += "etc/anki-version-base"
FILES_${PN} += "etc/anki-version-code"
FILES_${PN} += "etc/anki-version"
