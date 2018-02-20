FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}-${PV}:"
DEPENDS = "base-passwd"

SRC_URI_append += "file://selinux-fstab"
SRC_URI_append += "file://fstab"
SRC_URI_append += "file://systemd/cache.mount"
SRC_URI_append += "file://systemd/firmware.mount"
SRC_URI_append += "file://systemd/systemrw.mount"
SRC_URI_append += "file://systemd/dsp.mount"
SRC_URI_append += "file://systemd/media-card.mount"
SRC_URI_append += "file://systemd/media-ram.mount"
SRC_URI_append += "file://systemd/persist.mount"
SRC_URI_append += "file://systemd/proc-bus-usb.mount"
SRC_URI_append += "file://systemd/var-volatile.mount"
SRC_URI_append += "file://systemd/dash.mount"

dirs755 += "/media/cf /media/net /media/ram \
            /media/union /media/realroot /media/hdd \
            /media/mmc1"

dirs755_append_apq8053 +="/persist /cache /dsp "

#TODO Enabling systemd we need to add /firmware in dirs_755 list.
dirs755_append_apq8009 += "/firmware /persist /cache"
dirs755_append_apq8017 += "/firmware /persist /cache /dsp /systemrw"

do_install_append(){
    install -m 755 -o diag -g diag -d ${D}/media
    install -m 755 -o diag -g diag -d ${D}/mnt/sdcard
    if ${@base_contains('DISTRO_FEATURES','selinux','true','false',d)}; then
      install -m 0644 ${WORKDIR}/selinux-fstab ${D}${sysconfdir}/fstab
    else
      install -m 0644 ${WORKDIR}/fstab ${D}${sysconfdir}/fstab
    fi
    if ${@base_contains('DISTRO_FEATURES','systemd','true','false',d)}; then
     if ${@base_contains('DISTRO_FEATURES','nand-boot','false','true',d)}; then
      install -d 0644 ${D}${sysconfdir}/systemd/system
      install -m 0644 ${WORKDIR}/systemd/cache.mount ${D}${sysconfdir}/systemd/system/cache.mount
      install -m 0644 ${WORKDIR}/systemd/firmware.mount ${D}${sysconfdir}/systemd/system/firmware.mount
      install -m 0644 ${WORKDIR}/systemd/dsp.mount ${D}${sysconfdir}/systemd/system/dsp.mount
      install -m 0644 ${WORKDIR}/systemd/persist.mount ${D}${sysconfdir}/systemd/system/persist.mount
      install -d 0644 ${D}${sysconfdir}/systemd/system/local-fs.target.requires
      ln -sf  ../cache.mount  ${D}${sysconfdir}/systemd/system/local-fs.target.requires/cache.mount
      ln -sf  ../firmware.mount  ${D}${sysconfdir}/systemd/system/local-fs.target.requires/firmware.mount
      ln -sf  ../dsp.mount  ${D}${sysconfdir}/systemd/system/local-fs.target.requires/dsp.mount
      ln -sf  ../persist.mount  ${D}${sysconfdir}/systemd/system/local-fs.target.requires/persist.mount
      if [  ${BASEMACHINE} == "apq8017" ]; then
        install -m 0644 ${WORKDIR}/systemd/var-volatile.mount  ${D}${sysconfdir}/systemd/system/var-volatile.mount
        ln -sf  ../var-volatile.mount ${D}${sysconfdir}/systemd/system/local-fs.target.requires/var-volatile.mount
        install -m 0644 ${WORKDIR}/systemd/systemrw.mount ${D}${sysconfdir}/systemd/system/systemrw.mount
        ln -sf  ../systemrw.mount  ${D}${sysconfdir}/systemd/system/local-fs.target.requires/systemrw.mount
      fi
      if [  ${BASEMACHINE} == "apq8009" ]; then
        rm -rf  ${D}${sysconfdir}/systemd/system/local-fs.target.requires/dsp.mount
        rm -rf  ${D}${sysconfdir}/systemd/system/dsp.mount
      fi
     fi
    fi
    ln -s /mnt/sdcard ${D}/sdcard
    rmdir ${D}/tmp
    ln -s /var/tmp ${D}/tmp
    ln -s /var/run/resolv.conf ${D}/etc/resolv.conf
}
