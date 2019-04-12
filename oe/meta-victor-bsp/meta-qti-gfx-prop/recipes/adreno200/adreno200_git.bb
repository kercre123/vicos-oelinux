inherit qlicense qprebuilt

DESCRIPTION = "Adreno200"
DEPENDS += "glib-2.0"
DEPENDS += "virtual/kernel"
DEPENDS += "libxml-simple-perl-native"

S      = "${WORKDIR}/adreno200"
SRC_DIR = "${WORKSPACE}/adreno200/"

PR = "r0"

# Need the glib-2.0 headers
CFLAGS += "-I${STAGING_INCDIR}/glib-2.0"
CFLAGS += "-I${STAGING_LIBDIR}/glib-2.0/include"
CFLAGS += "-D__LE__"
LDFLAGS += "-lglib-2.0 -lpthread"

INCSUFFIX = "adreno200_ESX"

# Use RB driver for following targets
INCSUFFIX_apq8009 = "adreno200_RB"
INCSUFFIX_apq8017 = "adreno200_RB"

include ${INCSUFFIX}.inc
