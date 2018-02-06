inherit autotools qcommon qlicense qprebuilt

DESCRIPTION = "libsdmextension Library"
PR = "r3"

PACKAGES = "${PN}"

SRC_DIR = "${WORKSPACE}/display/display-noship/"
S = "${WORKDIR}/display/display-noship/"

PREBUILT = "1"

DEPENDS += "display-hal-linux"
DEPENDS += "drm"
DEPENDS += "libdrm"

EXTRA_OECONF = " --with-core-includes=${WORKSPACE}/system/core/include"
EXTRA_OECONF += " --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"

EXTRA_OECONF_append_apq8098 = " --enable-sdedrm"

LDFLAGS += "-llog -lutils -lcutils -lsdmutils -ldrm"
CPPFLAGS += "-DTARGET_HEADLESS"

CPPFLAGS += "-I${STAGING_INCDIR}/libdrm"
CPPFLAGS += "-I${WORKSPACE}/display/display-hal/include"
CPPFLAGS += "-I${WORKSPACE}/display/display-noship/hdr_tm"

# Need the display sdm headers
CPPFLAGS += "-I${STAGING_INCDIR}/sdm"

FILES_${PN} = "${libdir}/*.so"
