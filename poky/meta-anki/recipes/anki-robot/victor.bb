DESCRIPTION = "Victor Robot daemon"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://anki/victor"

SRCREV   = "${AUTOREV}"
S        = "${WORKDIR}/victor"
BUILDSRC = "${S}/_build/android/Release"
VICOSSRC = "${S}/_build/vicos/Release"

export SSH_AUTH_SOCK

do_package_qa[noexec] = "1"

do_compile () {
   cd ${S}
   echo "gitdir: ${WORKSPACE}/.git/modules/anki/victor" > .git
   source ./project/victor/envsetup.sh
   export TOPLEVEL=`gettop`
   ./project/victor/build-victor.sh -p android -c Release -F factoryTest
   ./project/victor/build-victor.sh -p vicos -c Release
}

do_install () {
    ${S}/project/victor/scripts/install.sh ${BUILDSRC} ${D}
    ${S}/project/victor/scripts/install.sh -k ${VICOSSRC} ${D}
}

FILES_${PN} += "anki/"
