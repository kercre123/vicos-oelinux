inherit cmake pkgconfig

SUMMARY = "JSON C++ lib used to read and write json file."
DESCRIPTION = "Jsoncpp is an implementation of a JSON (http://json.org) reader \
               and writer in C++. JSON (JavaScript Object Notation) is a \
               lightweight data-interchange format. It is easy for humans to \
               read and write. It is easy for machines to parse and generate."

HOMEPAGE = "http://sourceforge.net/projects/jsoncpp/"

LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=c56ee55c03a55f8105b969d8270632ce"

# The revision of the recipe used to build the package.
PR = "r0"

SRC_URI = "git://source.codeaurora.org/quic/la/platform/external/jsoncpp;protocol=http;branch=android-external.lnx.2.0-rel"

#	90c81b9c9aef09ef4ffb8de1779301734336d897
SRCREV = "90c81b9c9aef09ef4ffb8de1779301734336d897"

S = "${WORKDIR}/git"

EXTRA_OECMAKE += "-DJSONCPP_LIB_BUILD_SHARED=ON -DJSONCPP_WITH_TESTS=OFF"
