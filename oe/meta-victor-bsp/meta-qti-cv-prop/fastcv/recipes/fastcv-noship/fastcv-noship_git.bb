inherit qcommon qlicense qprebuilt sdllvm

DESCRIPTION = "Generates libfastcvopt"

FILESPATH =+ "${WORKSPACE}/cv/:"
SRC_URI = "file://fastcv-noship/"

SRC_DIR = "${WORKSPACE}/cv/fastcv-noship/"
S       = "${WORKDIR}/fastcv-noship"

DEPENDS += "adsprpc liblog adreno200 glib-2.0"

EXTRA_OECONF += " --with-glib"
EXTRA_OECONF += " --enable-target-${BASEMACHINE}=yes"
EXTRA_OECONF += " --with-cutils-headers=${STAGING_INCDIR}/cutils/"
EXTRA_OECONF += " --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/"

#Used to enable SDLLVM compiler
ENABLE_SDLLVM = "true"

#Uncomment this to use MDSP on LE for fastCV call
#EXTRA_OEMAKE += " MDSP_ENABLE="true""

CPPFLAGS += " -I${STAGING_INCDIR}"
CPPFLAGS += " -I${STAGING_INCDIR}/cutils"
CPPFLAGS += " -I${STAGING_INCDIR}/glib-2.0"
CPPFLAGS += " -I${STAGING_LIBDIR}/glib-2.0/include"
CPPFLAGS += " -I${STAGING_INCDIR}/adreno"
CPPFLAGS += " -DLE_ENABLE"

CPPFLAGS += " -include unistd.h"
CPPFLAGS += " -include sys/time.h"
CPPFLAGS += " -Wno-error"
CPPFLAGS += " -DLE_ENABLE"
CPPFLAGS += " -include stdint.h"
CPPFLAGS += " -Dstrlcpy=g_strlcpy"
CPPFLAGS += " -Dstrlcat=g_strlcat"
CPPFLAGS += " -include stdlib.h"
CPPFLAGS += " -include glib.h"
CPPFLAGS += " -include glibconfig.h"
CPPFLAGS += " -include properties.h"

LDFLAGS += " -L${STAGING_BASELIBDIR} -L${STAGING_LIBDIR} -L${STAGING_DIR_TARGET}"
LDFLAGS += " -lcutils"
LDFLAGS += " -lglib-2.0"
LDFLAGS += " -llog"

python __anonymous () {
    if d.getVar('BASEMACHINE', True) == 'apq8053':
        d.setVar('HEXAGON_VERSION', 'v50')
    elif d.getVar('BASEMACHINE', True) == 'apq8096':
        d.setVar('HEXAGON_VERSION', 'v60')
    elif d.getVar('BASEMACHINE', True) == 'apq8098':
        d.setVar('HEXAGON_VERSION', 'v62')
    else:
        d.setVar('HEXAGON_VERSION', 'v50')
}

do_install_append(){
    install -d ${D}/usr/lib/rfsa
    install -d ${D}/usr/lib/rfsa/adsp
    install -m 0644 ${S}/lib/DSP/${HEXAGON_VERSION}/libfastcvadsp.so ${D}/usr/lib/rfsa/adsp
    install -m 0644 ${S}/lib/DSP/${HEXAGON_VERSION}/libfastcvadsp_skel.so ${D}/usr/lib/rfsa/adsp
    install -m 0644 ${S}/lib/DSP/${HEXAGON_VERSION}/libdspCV_skel.so ${D}/usr/lib/rfsa/adsp
    install -m 0644 ${S}/lib/DSP/${HEXAGON_VERSION}/libapps_mem_heap.so ${D}/usr/lib/rfsa/adsp
}

#PACKAGES = "${PN}"
INSANE_SKIP_${PN} = "dev-so"
INSANE_SKIP_${PN} += "arch"
INSANE_SKIP_${PN} += "dev-deps"
INSANE_SKIP_${PN} += "already-stripped"

SOLIBS = ".so"
FILES_SOLIBSDEV = ""

FILES_${PN} += "/usr/lib/* ${libdir}/* "
