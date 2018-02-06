inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "GPS asn1c cper"
PR = "r1"

FILESPATH =+ "${WORKSPACE}:"
SRC_DIR = "${WORKSPACE}/gps-noship-external/osys/asn1c/rtpersrc/"
SRC_URI = "file://gps-noship-external/osys/asn1c/rtpersrc/"
S = "${WORKDIR}/gps-noship-external/osys/asn1c/rtpersrc"
DEPENDS = "glib-2.0 asn1c-rtx asn1c-crt"
EXTRA_OECONF = "--with-glib"
