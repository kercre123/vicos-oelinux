SUMMARY = "Anki Firmware Analyzer Tool"
DESCRIPTION = "The afat.sh script is designed to test various files in the rootfs."
PR = "r0"
SECTION = "security"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://Licenses/GPL-2;md5=94d55d512a9ba36caa9b7df079bae19f"

SRC_URI = "file://Licenses/GPL-2 \
	   file://afat.sh \
           file://data/binaries \
           file://data/conffiles \
           file://data/dbfiles \
           file://data/passfiles \
           file://data/patterns \
           file://data/sshfiles \
           file://data/sslfiles \
           file://data/webservers \
           "

# SRC_URI[md5sum] = "57cc3fbbbe48e8ebd4672c569954374d"
# SRC_URI[sha256sum] = "05822cd8668589038d20650faa0e56f740911d8ad06f7005b3d12a5c76591b90"


S = "${WORKDIR}"

do_install() {
    install -d ${D}${bindir}/
    install -d ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/afat.sh    ${D}${bindir}
    install -m 0755 ${WORKDIR}/data/binaries    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/conffiles    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/dbfiles    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/passfiles    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/patterns    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/sshfiles    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/sslfiles    ${D}${bindir}/data
    install -m 0755 ${WORKDIR}/data/webservers    ${D}${bindir}/data
    sed -i 's/\r//' ${D}${bindir}/afat.sh
}

RDEPENDS_${PN} = "bash binutils"

BBCLASSEXTEND = "native"

