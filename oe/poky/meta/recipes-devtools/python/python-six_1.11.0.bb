SUMMARY = "Python 2 and 3 compatibility library"
SECTION = "devel/python"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=35cec5bf04dd0820d0a18533ea7c774a"

SRC_URI = "https://github.com/benjaminp/six/archive/${PV}.tar.gz"

SRC_URI[md5sum] = "72cba57bb6b67190303c49aa33e07b52"
SRC_URI[sha256sum] = "927dc6fcfccd4e32e1ce161a20bf8cda39d8c9d5f7a845774486907178f69bd4"

S = "${WORKDIR}/six-${PV}"

inherit setuptools

RDEPENDS_${PN} = ""
RDEPENDS_${PN}_class-native = ""

BBCLASSEXTEND = "native nativesdk"
