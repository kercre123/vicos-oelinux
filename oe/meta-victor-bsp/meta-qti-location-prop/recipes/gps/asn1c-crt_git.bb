inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS asn1c"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship-external/osys/asn1c/rtsrc/"
SRC_URI = "file://gps-noship-external/osys/asn1c/rtsrc/"
S = "${WORKDIR}/gps-noship-external/osys/asn1c/rtsrc"
DEPENDS = "glib-2.0 libcutils"
EXTRA_OECONF = "--with-glib"
