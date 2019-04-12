inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS asn1c"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship-external/osys/asn1c/rtxsrc/"
SRC_URI = "file://gps-noship-external/osys/asn1c/rtxsrc/"
S = "${WORKDIR}/gps-noship-external/osys/asn1c/rtxsrc"
DEPENDS = "glib-2.0 libcutils"
EXTRA_OECONF = "--with-glib"
