DESCRIPTION = "Victor Cloud Services daemon"
LICENSE = "Anki-Inc.-Proprietary"                                                                   
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\                           
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

SERVICE_FILE = "vic-cloud.service"

SRC_URI = "file://${SERVICE_FILE}"

inherit systemd

do_install_append () {
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${D}${systemd_unitdir}/system/
       install -m 0644 ${WORKDIR}/${SERVICE_FILE} -D ${D}${systemd_unitdir}/system/${SERVICE_FILE}
   fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
SYSTEMD_SERVICE_${PN} = "${SERVICE_FILE}"

do_compile() {
    docker build -t armbuilder ${S}/vector-cloud/docker-builder/.
}

GOPATH = "~/.local/go"

inherit externalsrc

EXTERNALSRC = "${WORKSPACE}/anki/vector-cloud"

GID_ANKI      = '2901'
GID_CLOUD     = '888'
GID_ANKINET   = '2905'

UID_NET       = "${GID_ANKINET}"
UID_CLOUD     = "${GID_CLOUD}"

do_compile() {
    cd "${EXTERNALSRC}"
    export GOPATH="${GOPATH}"
    make all
}

do_install () {
    mkdir -p ${D}/anki/bin
    cp ${WORKSPACE}/anki/vector-cloud/build/* ${D}/anki/bin
}

FILES_${PN} += "anki/bin/vic-cloud"
FILES_${PN} += "anki/bin/vic-gateway"
