inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "externaldr "
PR = "r1"

SRC_DIR = "${WORKSPACE}/gps-noship/dr_amt/dr_glue/"
S = "${WORKDIR}/gps-noship/dr_amt/dr_glue"
DEPENDS = "glib-2.0 gps-utils lbs-core loc-pla loc-stub loc-externaldrcore gnsspps diag"
EXTRA_OECONF = "--with-glib \
                --enable-a6dof=yes \
                --enable-qdr2_customer1=no \
               "

