SUMMARY = "Toybox combines common utilities together into a single executable."
HOMEPAGE = "http://www.landley.net/toybox/"

# AOSP Toybox release 0.7.6
PV = "0.7.6-android-o-mr1-iot-preview-7+git${SRCPV}"
SRCREV = "14cafec54042565de03bdad44441b3c57b7f524e"
SRC_URI = "git://android.googlesource.com/platform/external/toybox;protocol=https;branch=master"

S = "${WORKDIR}/git"

LICENSE = "BSD-0-Clause"
LIC_FILES_CHKSUM = "file://LICENSE;md5=f0b8b3dd6431bcaa245da0a08bd0d511"

SECTION = "base"

TOYBOX_BIN = "generated/unstripped/toybox"
RDEPENDS_${PN} = "zlib libattr"

do_configure() {
    [[ -f .config ]] && echo "Found .config"
    # Do not generate defconfig - use the .config checked into git
    #oe_runmake defconfig

    # Not necessary - already disable in .config checked into git
    # Disable killall5 as it isn't managed by update-alternatives
    #sed -e 's/CONFIG_KILLALL5=y/# CONFIG_KILLALL5 is not set/' -i .config

    sed -e 's/CONFIG_TOYBOX_SELINUX=y/# CONFIG_TOYBOX_SELINUX is not set/' -i .config
    sed -e 's/CONFIG_CHCON=y/# CONFIG_CHCON is not set/' -i .config

    # Disable all the Android stuff 
    sed -e 's/CONFIG_TOYBOX_ON_ANDROID=y/# CONFIG_TOYBOX_ON_ANDROID is not set/' -i .config
    sed -e 's/CONFIG_TOYBOX_ANDROID_SCHEDPOLICY=y/# CONFIG_TOYBOX_ANDROID_SCHEDPOLICY is not set/' -i .config
    sed -e 's/CONFIG_GETENFORCE=y/# CONFIG_GETENFORCE is not set/' -i .config
    sed -e 's/CONFIG_GETPROP=y/# CONFIG_GETPROP is not set/' -i .config
    sed -e 's/CONFIG_LOAD_POLICY=y/# CONFIG_LOAD_POLICY is not set/' -i .config
    sed -e 's/CONFIG_LOG=y/# CONFIG_LOG is not set/' -i .config
    sed -e 's/CONFIG_RESTORECON=y/# CONFIG_RESTORECON is not set/' -i .config
    sed -e 's/CONFIG_RUNCON=y/# CONFIG_RUNCON is not set/' -i .config
    sed -e 's/CONFIG_SENDEVENT=y/# CONFIG_SENDEVENT is not set/' -i .config
    sed -e 's/CONFIG_SETENFORCE=y/# CONFIG_SETENFORCE is not set/' -i .config
    sed -e 's/CONFIG_SETPROP=y/# CONFIG_SETPROP is not set/' -i .config
    sed -e 's/CONFIG_START=y/# CONFIG_START is not set/' -i .config
    sed -e 's/CONFIG_STOP=y/# CONFIG_STOP is not set/' -i .config

}

do_compile() {
    oe_runmake ${TOYBOX_BIN}

    # Create a list of links needed
    ${BUILD_CC} -I . scripts/install.c -o generated/instlist
    ./generated/instlist long | sed -e 's#^#/#' > toybox.links
}

do_install() {
    # Install manually instead of using 'make install'
    install -d ${D}${base_bindir}
    if grep -q "CONFIG_TOYBOX_SUID=y" ${B}/.config; then
        install -m 4755 ${B}/${TOYBOX_BIN} ${D}${base_bindir}/toybox
    else
        install -m 0755 ${B}/${TOYBOX_BIN} ${D}${base_bindir}/toybox
    fi

    install -d ${D}${sysconfdir}
    install -m 0644 ${B}/toybox.links ${D}${sysconfdir}
}

inherit update-alternatives

# If you've chosen to install toybox you probably want it to take precedence
# over busybox and coreutils where possible but not over other packages
ALTERNATIVE_PRIORITY = "110"

python do_package_prepend () {
    # Read links from /etc/toybox.links and create appropriate
    # update-alternatives variables

    dvar = d.getVar('D', True)
    pn = d.getVar('PN', True)
    target = "/bin/toybox"

    f = open('%s/etc/toybox.links' % (dvar), 'r')
    for alt_link_name in f:
        alt_link_name = alt_link_name.strip()
        alt_name = os.path.basename(alt_link_name)
        d.appendVar('ALTERNATIVE_%s' % (pn), ' ' + alt_name)
        d.setVarFlag('ALTERNATIVE_LINK_NAME', alt_name, alt_link_name)
        d.setVarFlag('ALTERNATIVE_TARGET', alt_name, target)
    f.close()
}
