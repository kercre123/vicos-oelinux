inherit autotools pkgconfig qlicense
DESCRIPTION = "video utils"

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://mm-video-utils/"

S = "${WORKDIR}/mm-video-utils"

DEPENDS += "adreno200"
DEPENDS += "libhardware"
DEPENDS += "native-frameworks"
DEPENDS += "media"
DEPENDS += "libxml2"
DEPENDS += "system-core"
DEPENDS += "display-hal"
DEPENDS += "mm-video-noship"

# configure features
EXTRA_OECONF_append =" --enable-use-glib="yes""
EXTRA_OECONF_append =" --enable-target-${SOC_FAMILY}="yes""
EXTRA_OECONF_append =" --with-glib"
EXTRA_OECONF_append =" --with-glib-headers=${STAGING_INCDIR}/glib-2.0/"
EXTRA_OECONF_append =" --with-glib-lib-dir=${STAGING_LIBDIR}/glib-2.0/include"
EXTRA_OECONF_append =" --with-omx-headers=${STAGING_INCDIR}/mm-core"
EXTRA_OECONF_append =" --with-xml2-headers=${STAGING_INCDIR}/libxml2"
EXTRA_OECONF_append =" --with-common-headers=${STAGING_INCDIR}/"
EXTRA_OECONF_append =" --with-android-headers=${STAGING_INCDIR}/media/hardware/"
EXTRA_OECONF_append =" --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
EXTRA_OECONF_append =" --with-time-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/uapi/linux/"
EXTRA_OECONF_append =" --with-graphics-headers=${STAGING_INCDIR}/system"
EXTRA_OECONF_append =" --with-stagefright-headers=${STAGING_INCDIR}/libstagefrighthw/"
EXTRA_OECONF_append =" --with-streamparser-headers=${STAGING_INCDIR}/mm-video/streamparser/"
EXTRA_OECONF_append =" --with-fastcrc-headers=${STAGING_INCDIR}/mm-video/fastcrc/"
EXTRA_OECONF_append =" --with-utils-headers=${STAGING_INCDIR}/mm-video/utils/"

EXTRA_OECONF_append =" --with-kernel-media-headers=${STAGING_KERNEL_BUILDDIR}/usr/include/media"

EXTRA_OEMAKE += "BOARD_USES_ADRENO=false"

export TARGET_LIBRARY_SUPPRESS_LIST="libui libgui libbinder libxml2 libOmxCore"

