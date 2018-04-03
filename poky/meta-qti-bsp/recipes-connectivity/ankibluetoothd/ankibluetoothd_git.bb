inherit autotools pkgconfig systemd

DESCRIPTION = "Anki Bluetooth Daemon"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

CPPFLAGS_append = " -DUSE_ANDROID_LOGGING "
CFLAGS_append = " -DUSE_ANDROID_LOGGING "
LDFLAGS_append = " -llog "

FILESPATH =+ "${WORKSPACE}:"
SRC_URI = "file://external/anki-ble/"
SRC_URI += "file://ankibluetoothd.service"
SRC_URI += "file://vicdevsetup.service"

S = "${WORKDIR}/external/anki-ble/"

DEPENDS += "btvendorhal libhardware bt-property"

do_install_append() {
  if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
    install -d ${D}${sysconfdir}/systemd/system/
    install -m 0644 ${WORKDIR}/ankibluetoothd.service \
      -D ${D}${sysconfdir}/systemd/system/ankibluetoothd.service
    install -m 0644 ${WORKDIR}/vicdevsetup.service \
      -D ${D}${sysconfdir}/systemd/system/vicdevsetup.service
    install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants/
    ln -sf /etc/systemd/system/ankibluetoothd.service \
      ${D}${sysconfdir}/systemd/system/multi-user.target.wants/ankibluetoothd.service
  fi
}

FILES_${PN} += "${systemd_unitdir}/system/"
