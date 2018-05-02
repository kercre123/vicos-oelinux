DESCRIPTION = "Anki Version Number"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"                                                                        
SRC_URI = "file://ANKI_VERSION"

do_install_append () {
    install -d ${D}/etc
    install -m 0444 ${WORKDIR}/ANKI_VERSION ${D}/etc/os-version-base

    # ANKI_BUILD_VERSION will be set by TeamCity for CI builds, default to 0 for developer builds    
    : ${ANKI_BUILD_VERSION:=0}
    echo "${ANKI_BUILD_VERSION}" > ${D}/etc/os-version-code
    chmod 0444 ${D}/etc/os-version-code

    if [ -z ${ANKI_BUILD_REVISION} ]; then
        GIT=`which git`                                                                                  
        if [ ! -z $GIT ]; then                                                                           
            ANKI_BUILD_REVISION=`git rev-parse --short HEAD`                                                              
        else
            ANKI_BUILD_REVISION=""
        fi
    fi    
    echo "${ANKI_BUILD_REVISION}" > ${D}/etc/os-version-rev
    echo 0444 ${D}/etc/os-version-rev

    # build type tag 
    if [[ ${USER_BUILD} != "1" ]]; then
        # set to "d" for dev builds
        ANKI_BUILD_TYPE="d"
    else
        # empty for user (release) builds
        ANKI_BUILD_TYPE=""
    fi

    BASE_VERSION=$(cat ${WORKDIR}/ANKI_VERSION)
    echo "${BASE_VERSION}.${ANKI_BUILD_VERSION}${ANKI_BUILD_TYPE}" > ${D}/etc/os-version
    chmod 0444 ${D}/etc/os-version
}

FILES_${PN} += "etc/os-version-base"
FILES_${PN} += "etc/os-version-code"
FILES_${PN} += "etc/os-version-rev"
FILES_${PN} += "etc/os-version"
