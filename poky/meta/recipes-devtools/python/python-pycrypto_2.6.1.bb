DESCRIPTION = "Cryptographic modules for Python."
HOMEPAGE = "http://www.pycrypto.org/"
SECTION = "devel/python"
LICENSE = "PSFv2"
LIC_FILES_CHKSUM = "file://COPYRIGHT;md5=35f354d199e8cb7667b059a23578e63d"

SRC_URI = "https://ftp.dlitz.net/pub/dlitz/crypto/pycrypto/pycrypto-2.6.1.tar.gz"
S = "${WORKDIR}/pycrypto-2.6.1"

SRC_URI += " \
    file://cross-compiling.patch \
"

SRC_URI[md5sum] = "55a61a054aa66812daf5161a0d5d7eda"
SRC_URI[sha256sum] = "f2ce1e989b272cfcb677616763e0a2e7ec659effa67a88aa92b3a65528f60a3c"

inherit setuptools

DEPENDS += " gmp"

RDEPENDS_${PN} = ""
RDEPENDS_${PN}_class-native = ""

BBCLASSEXTEND = "native nativesdk"
