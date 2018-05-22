DESCRIPTION = "Victor Robot daemon"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://anki/victor"

SRCREV   = "${AUTOREV}"
S        = "${WORKDIR}/victor"

SERVICE_FILE = "anki-menuman.service"                                                               
                                                                                                    
SRC_URI += "file://${SERVICE_FILE}"                                                                  
                                                                                                    
inherit systemd                                                                                     

export SSH_AUTH_SOCK

do_package_qa[noexec] = "1"

do_compile () {
   cd ${S}
   echo "gitdir: ${WORKSPACE}/.git/modules/anki/victor" > .git
   source ./project/victor/envsetup.sh
   export TOPLEVEL=`gettop`

   pushd robot/fixture/helpware
   make menuman
   popd
}

do_install () {
    # install menuman
    install -d ${D}/anki/menuman
    install -m 0755 ${S}/robot/fixture/helpware/menuman ${D}/anki/menuman/menuman
    install -m 0755 ${S}/robot/fixture/helpware/files/* ${D}/anki/menuman/

    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then                 
        install -d ${D}${systemd_unitdir}/system/                                                    
        install -m 0644 ${WORKDIR}/${SERVICE_FILE} -D ${D}${systemd_unitdir}/system/${SERVICE_FILE}  
    fi

    # install fake version numbers
    install -d ${D}/anki/etc
    echo "0" > ${D}/anki/etc/version
    echo "0000000" > ${D}/anki/etc/revision
}

FILES_${PN} += "anki/menuman"
FILES_${PN} += "anki/etc"
FILES_${PN} += "${systemd_unitdir}/system/"                                                         
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"
