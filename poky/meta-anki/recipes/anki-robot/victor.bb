DESCRIPTION = "Victor Robot daemon"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://anki/victor"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/victor"

#inherit systemd

do_compile () {
   PSEUDO_UNLOAD=1 bash
   cd ${S}
   source ./project/victor/envsetup.sh
   echo "gettop = $(gettop)"
   export TOPLEVEL=`gettop`
   ./project/victor/build-victor.sh -p android -c Release -F factoryTest
}
