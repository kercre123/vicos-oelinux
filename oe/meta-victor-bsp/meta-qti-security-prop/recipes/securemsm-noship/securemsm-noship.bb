inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "securemsmnoship with QseecomAPI user space library to interact with qseecom driver"
DEPENDS = "virtual/kernel glib-2.0 linux-libc-headers securemsm-initial time-services"

SRC_DIR = "${WORKSPACE}/security/securemsm-noship/"
S = "${WORKDIR}/security/securemsm-noship/"

PR = "1"

INSANE_SKIP_${PN} = "dev-so"
INSANE_SKIP_${PN} += "dev-deps"
INSANE_SKIP_${PN} += "debug-files"

PREBUILT = "1"

EXTRA_OEMAKE += "ARCH=${TARGET_ARCH} CROSS_COMPILE=${TARGET_PREFIX}"

EXTRA_OECONF += "--with-kernel=${STAGING_KERNEL_DIR} \
                --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"
FILES_${PN} += "/usr/bin/*"
FILES_${PN} += "${bindir}/*"

FILES_${PN} += "${libdir}"

# By default .so libs(which are not versioned) are treated as development libraries
# which are not packaged in the release package but instead in the development package
# (securemsm-noship-dev). The following line explicitly overrides what goes in
# the dev package so that anything remaining can go in the release package
# like the .so libs
FILES_${PN}-dev = "${libdir}/*.la"

do_install() {
   mkdir -p ${D}/usr/bin
   cp -pPr ${S}${WORKDIR}/image/usr/bin/hdcp* ${D}/usr/bin
   }
