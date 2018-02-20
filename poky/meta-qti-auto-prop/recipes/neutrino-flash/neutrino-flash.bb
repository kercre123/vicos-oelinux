inherit autotools qcommon qlicense qprebuilt
DESCRIPTION = "Neutrino Flashloader"

FILESEXTRAPATHS_prepend := "${WORKSPACE}/:"
SRC_DIR = "${WORKSPACE}/neutrino-noship/"
S =  "${WORKDIR}/neutrino-noship/"

EXTRA_OEMAKE = 'CC="${CC}" AR="${AR}" RANLIB="${RANLIB}" CFLAGS="${CFLAGS}" \
                LDFLAGS="${LDFLAGS}" LD="${LD}" OS="${TARGET_SYS}" \
                TARGET="${TARGET_OS}" BASE="${prefix}" MANDIR="${mandir}"'


do_compile () {
        oe_runmake -C flash-loader
}

do_install () {
    install -d ${D}${bindir}
    install -m 0755 ${S}/flash-loader/ntn_flashloder ${D}${bindir}
}
