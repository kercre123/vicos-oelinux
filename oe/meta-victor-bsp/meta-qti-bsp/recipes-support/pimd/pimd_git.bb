inherit autotools-brokensep linux-kernel-base
DESCRIPTION = "PIMD - Multicast Routing Daemon"
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://LICENSE;md5=94f108f91fab720d62425770b70dd790"

PR = "r5"
do_configure() {
    :
}

SRCREV = "c4b1c9f4b5eaa70931d0f62f456ae10ac4c4a829"
SRC_URI = "git://codeaurora.org/quic/le/pimd.git;protocol=git;branch=github/master \
           file://0001-pimb-multicast-support-on-network.patch \
           file://no-deprecated-declarations.patch \
"

SRC_URI_append_9615-cdp += " \
           file://defs_fix_multicast_subnetmask_on_rmnet.patch \
           file://vif_fix_multicast_subnetmask_on_rmnet.patch \
           file://pimd.conf \
           file://config_fix_multicast_subnetmask_on_rmnet.patch "

S = "${WORKDIR}/git"

do_compile() {
  make
}

do_install() {
        make install DESTDIR=${D}
}
do_install_append_9615-cdp() {
    install -m 0755 ${WORKDIR}/pimd.conf ${D}${sysconfdir}
}
