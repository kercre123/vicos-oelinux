SUMMARY = "Secure parasitic rdate replacement"

DESCRIPTION = "\
tlsdate sets the local clock by securely connecting with TLS to remote \
servers and extracting the remote time out of the secure handshake. Unlike \
ntpdate, tlsdate uses TCP, for instance connecting to a remote HTTPS or TLS \
enabled service, and provides some protection against adversaries that try to \
feed you malicious time information"

HOMEPAGE = "https://github.com/ioerror/tlsdate"
SECTION = "console/network"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://LICENSE;md5=99cc53e1ea4c7781f0160aa3ae25e8b8"
LIBTOOL = "${HOST_SYS}-libtool"
inherit autotools-brokensep

SRC_URI = "git://github.com/ioerror/tlsdate.git"
SRCREV = "ae396da167a9e43ce10c2db0956fb2e2b0d400ea"

S = "${WORKDIR}/git"

DEPENDS += "libevent"

RDEPENDS_${PN} = "openssl libtool libevent"

do_configure() {
    mkdir -p config
    autoreconf --install --verbose --force
    oe_runconf
}
