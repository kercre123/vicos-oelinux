inherit autotools pkgconfig qlicense qprebuilt sdllvm

DESCRIPTION = "MM Camera libraries for MSM/QSD"
SECTION  = "camera"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI   = "file://camera/services/mm-still"
SRC_DIR = "${WORKSPACE}/camera/services/mm-still"

SRCREV = "${AUTOREV}"
S      = "${WORKDIR}/camera/mm-still"

DEPENDS += "glib-2.0 libxml2 mm-camera camera"

ENABLE_SDLLVM = "true"
TARGET_CFLAGS +="${THUMB_FLAGS} ${OPTIONAL_CFLAGS}"
TARGET_LDFLAGS +="${OPTIONAL_LDFLAGS} -avoid-version"

def get_platform(d):
    if d.getVar('BASEMACHINE', True) == 'apq8053':
        return "TARGET_BOARD_PLATFORM=8953"
    elif d.getVar('BASEMACHINE', True) == 'apq8096':
        return "TARGET_BOARD_PLATFORM=8996"
    else:
        return ""

def config_camera_debug_data(d):
    if d.getVar('BASEMACHINE', True) == 'apq8096' and d.getVar('PRODUCT', True) == 'drone':
        return "CAMERA_DEBUG_DATA=true"
    else:
        return ""

EXTRA_OECONF += "--with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include \
                --with-glib \
                --with-common-includes=${STAGING_INCDIR} \
                --with-camera-hal-path=${WORKSPACE}/camera/lib \
                --with-mm-camera-path=${WORKSPACE}/camera/services/mm-camera/mm-camera2 \
                --with-mm-camerasdk=${WORKSPACE}/camera/services/mm-camerasdk"

EXTRA_OECONF += "${@get_platform(d)}"
EXTRA_OECONF += "TARGET_USES_ION=true"
EXTRA_OECONF += "CHROMATIX_VERSION=0309"
EXTRA_OECONF += "NEW_LOG_API=true"
EXTRA_OECONF += "JPEC_ENC=hw_sw"
EXTRA_OECONF += "USES_ARMV7=true"
EXTRA_OECONF += "FACT_VER=codecB"
EXTRA_OECONF += "${@config_camera_debug_data(d)}"

PACKAGES = "${PN}"
FILES_${PN}      = "${libdir}/*.so ${libdir}/*.so.* ${sysconfdir}/* ${libdir}/pkgconfig/* ${bindir}/*"
FILES_${PN}-dbg  = "${libdir}/.debug/*"
FILES_${PN}-dev  = "${libdir}/*.la ${includedir}"
INSANE_SKIP_${PN} = "dev-so"
