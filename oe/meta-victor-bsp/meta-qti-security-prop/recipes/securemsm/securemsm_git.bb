inherit qcommon autotools qlicense
DESCRIPTION = "Securemsm library with sampleclient used to test sampleapp with qseecom driver through QSEEComApi library"

DEPENDS = "securemsm-noship securemsm-initial system-core virtual/kernel glib-2.0 glibc"

SRC_DIR = "${WORKSPACE}/security/securemsm/"
S = "${WORKDIR}/security/securemsm/"

PR = "r1"

EXTRA_OECONF += " --with-kernel=${STAGING_KERNEL_DIR} \
                  --with-sanitized-headers=${STAGING_KERNEL_BUILDDIR}/usr/include"

# Append --with-rpmb to enable rpmb listener in qseecomd
EXTRA_OECONF_append_msm = " --with-rpmb"

FILES_${PN} += "/usr/bin/*"
FILES_${PN} += "${bindir}/*"

# This package contains symlinks that trip up insane
INSANE_SKIP_${PN} += "debug-files"

CFLAGS += "-I${STAGING_INCDIR}/cutils"

do_compile_prepend () {
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${libdir}
}

do_install() {
    install -m 0755 -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/security/securemsm/sampleclient/qseecom_sample_client ${D}${bindir}
    install -m 0755 ${WORKDIR}/security/securemsm/daemon/qseecomd ${D}${bindir}
}

