SUMMARY = "NetworkManager"
HOMEPAGE = "https://wiki.gnome.org/Projects/NetworkManager"
SECTION = "net/misc"

LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://COPYING;md5=cbbffd568227ada506640fe950a4823b \
                    file://libnm-util/COPYING;md5=1c4fa765d6eb3cd2fbd84344a1b816cd \
                    file://docs/api/html/license.html;md5=ac20f1edc24f72480a1106871e9fbe9a \
"

PROVIDES += "libnm"

DEPENDS = " \
    intltool-native \
    libnl \
    dbus \
    dbus-glib \
    dbus-glib-native \
    libgudev \
    util-linux \
    libndp  \
    readline \
    curl \
"


inherit gnomebase gettext systemd bluetooth gtk-doc

SRC_URI = " \
    ${GNOME_MIRROR}/NetworkManager/${@gnome_verdir("${PV}")}/NetworkManager-${PV}.tar.xz \
    file://NetworkManager.service \
    file://NetworkManager-dispatcher.service \
    file://NetworkManager.conf \
    file://udhcpd.conf \
    file://pre-down.d/0-udhcpd \
    file://pre-up.d/0-udhcpd \
    file://0001-sd-lldp.h-Remove-net-ethernet.h-seems-to-be-over-spe.patch \
    file://0002-Fixed-configure.ac-Fix-pkgconfig-sysroot-locations.patch \
"
SRC_URI[md5sum] = "94d02b80b120f166927e6ef242b29a9b"
SRC_URI[sha256sum] = "6be06ff93a05f3ee4da9e58e4a0d974eef245c08b6f02b00a9e44154c9801a26"



S = "${WORKDIR}/NetworkManager-${PV}"

EXTRA_OECONF = " \
    --disable-ifcfg-rh \
    --disable-ifnet \
    --disable-ifcfg-suse \
    --disable-more-warnings \
    --with-iptables=${sbindir}/iptables \
    --with-tests=no \
    --with-nmtui=no \
    --enable-vala=no \
    --disable-gtk-doc \
    --disable-gtk-doc-html \
    --enable-introspection=no \
    --disable-qt \
    --enable-ovs=no \
    --with-resolvconf=no \
    --disable-polkit-agent \
    --enable-polkit=disabled \
    --with-nmcli=no\
    --with-selinux=no \
    --with-dhcpcd=no \
"

# gobject-introspection related
GI_DATA_ENABLED_libc = "False"

# stolen from https://github.com/voidlinux/void-packages/blob/master/srcpkgs/NetworkManager/template
CFLAGS_libc-musl_append = " \
    -DHAVE_SECURE_GETENV -Dsecure_getenv=getenv \
    -D__USE_POSIX199309 -DRTLD_DEEPBIND=0 \
"

do_compile_prepend() {
    export GIR_EXTRA_LIBS_PATH="${B}/libnm/.libs:${B}/libnm-glib/.libs:${B}/libnm-util/.libs"
}

PACKAGECONFIG ??= "wifi nss systemd"

PACKAGECONFIG[systemd] = "--with-systemdsystemunitdir=${systemd_unitdir}/system, --without-systemdsystemunitdir"
PACKAGECONFIG[bluez5] = "--enable-bluez5-dun,--disable-bluez5-dun,bluez5"
# consolekit is not picked by shlibs, so add it to RDEPENDS too
PACKAGECONFIG[consolekit] = "--with-session-tracking=consolekit,,consolekit,consolekit"
PACKAGECONFIG[modemmanager] = "--with-modem-manager-1=yes,--with-modem-manager-1=no,modemmanager"
PACKAGECONFIG[ppp] = "--enable-ppp,--disable-ppp,ppp,ppp"
# Use full featured dhcp client instead of internal one
PACKAGECONFIG[dhclient] = "--with-dhclient=${base_sbindir}/dhclient,,,dhcp-client"
PACKAGECONFIG[dnsmasq] = "--with-dnsmasq=${bindir}/dnsmasq"
PACKAGECONFIG[nss] = "--with-crypto=nss,,nss"
PACKAGECONFIG[gnutls] = "--with-crypto=gnutls,,gnutls"
PACKAGECONFIG[wifi] = "--enable-wifi=yes,--enable-wifi=no,,wpa-supplicant"
PACKAGECONFIG[ifupdown] = "--enable-ifupdown,--disable-ifupdown"
PACKAGECONFIG[netconfig] = "--with-netconfig=yes,--with-netconfig=no"
PACKAGECONFIG[qt4-x11-free] = "--enable-qt,--disable-qt,qt4-x11-free"

PACKAGES =+ "libnmutil libnmglib libnm libnmglib-vpn \
  ${PN}-nmtui ${PN}-nmtui-doc \
  ${PN}-adsl \
"

FILES_libnmutil += "${libdir}/libnm-util.so.*"
FILES_libnmglib += "${libdir}/libnm-glib.so.*"
FILES_libnmglib-vpn += "${libdir}/libnm-glib-vpn.so.*"
FILES_libnm += "${libdir}/libnm.so.*"

FILES_${PN}-adsl = "${libdir}/NetworkManager/libnm-device-plugin-adsl.so"

FILES_${PN} += " \
    ${libexecdir} \
    ${libdir}/pppd/*/nm-pppd-plugin.so \
    ${libdir}/NetworkManager/${PV}/*.so \
    ${nonarch_libdir}/NetworkManager/VPN \
    ${nonarch_libdir}/NetworkManager/udhcpd.d \
    ${datadir}/polkit-1 \
    ${datadir}/dbus-1 \
    ${base_libdir}/udev/* \
    ${systemd_unitdir}/system \
"

RRECOMMENDS_${PN} += "iptables "

RCONFLICTS_${PN} = "connman"

FILES_${PN}-dev += " \
    ${datadir}/NetworkManager/gdb-cmd \
    ${libdir}/pppd/*/*.la \
    ${libdir}/NetworkManager/*.la \
"

FILES_${PN}-nmtui = " \
    ${bindir}/nmtui \
    ${bindir}/nmtui-edit \
    ${bindir}/nmtui-connect \
    ${bindir}/nmtui-hostname \
"

FILES_${PN}-nmtui-doc = " \
    ${mandir}/man1/nmtui* \
"

SYSTEMD_SERVICE_${PN} = "NetworkManager.service NetworkManager-dispatcher.service"

do_install_append() {
    rm -rf ${D}/run ${D}${localstatedir}/run
   if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
       install -d ${D}${systemd_unitdir}/system/
       install -m 0644 ${WORKDIR}/NetworkManager.service -D ${D}${systemd_unitdir}/system/NetworkManager.service
       install -m 0644 ${WORKDIR}/NetworkManager-dispatcher.service -D ${D}${systemd_unitdir}/system/NetworkManager-dispatcher.service
   fi

   install -d ${D}${sysconfdir}/NetworkManager/
   install -m 0644 ${WORKDIR}/NetworkManager.conf -D ${D}${sysconfdir}/NetworkManager/NetworkManager.conf

   install -d ${D}${sysconfdir}/NetworkManager/dispatcher.d/pre-down.d
   install -d ${D}${sysconfdir}/NetworkManager/dispatcher.d/pre-up.d
   install -d ${D}${sysconfdir}/NetworkManager/udhcpd.d/

   install -m 0500 ${WORKDIR}/udhcpd.conf -D ${D}${sysconfdir}/NetworkManager/udhcpd.d/udhcpd.conf
   install -m 0500 ${WORKDIR}/pre-down.d/0-udhcpd -D ${D}${sysconfdir}/NetworkManager/dispatcher.d/pre-down.d/
   install -m 0500 ${WORKDIR}/pre-up.d/0-udhcpd -D ${D}${sysconfdir}/NetworkManager/dispatcher.d/pre-up.d/

   rm -rf ${D}${sysconfdir}/NetworkManager/system-connections
   ln -s /data/etc/NetworkManager/system-connections ${D}${sysconfdir}/NetworkManager
}
