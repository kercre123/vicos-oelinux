FILESEXTRAPATHS_prepend := "${THISDIR}/systemd:"

SRC_URI += "file://1001-systemd-Disable-unused-mount-points.patch"
SRC_URI += "file://1002-systemd-Use-data-etc-localtime.patch"
SRC_URI += "file://1003-anki-Backport-env-status-for-ExecStop-ExecStopPost-c.patch"
SRC_URI += "file://mountpartitions.rules"
SRC_URI += "file://systemd-udevd.service"
SRC_URI += "file://ffbm.target"
SRC_URI += "file://mount-data"
SRC_URI += "file://mount-factory-data"
SRC_URI += "file://mount-data.service"
SRC_URI += "file://mtpserver.rules"
SRC_URI += "file://setup_localtime_link"
SRC_URI += "file://setup_localtime_link.service"

DEPENDS += "emr-cat blkdiscard user-data-locker"

EXTRA_OECONF += " --disable-efi"

# In aarch64 targets systemd is not booting with -finline-functions -finline-limit=64 optimizations
# So temporarily revert to default optimizations for systemd.
FULL_OPTIMIZATION = "-O2 -fexpensive-optimizations -frename-registers -fomit-frame-pointer -ftree-vectorize"

# Place systemd-udevd.service in /etc/systemd/system
do_install_append () {
   install -d ${D}/etc/systemd/system/
   install -d ${D}/lib/systemd/system/ffbm.target.wants
   install -d ${D}/etc/systemd/system/ffbm.target.wants
   rm ${D}/lib/udev/rules.d/60-persistent-v4l.rules
   install -m 0644 ${WORKDIR}/systemd-udevd.service \
       -D ${D}/etc/systemd/system/systemd-udevd.service
   install -m 0644 ${WORKDIR}/ffbm.target \
       -D ${D}/etc/systemd/system/ffbm.target
   # Enable logind/getty/password-wall service in FFBM mode
   ln -sf /lib/systemd/system/systemd-logind.service ${D}/lib/systemd/system/ffbm.target.wants/systemd-logind.service
   ln -sf /lib/systemd/system/getty.target ${D}/lib/systemd/system/ffbm.target.wants/getty.target
   ln -sf /lib/systemd/system/systemd-ask-password-wall.path ${D}/lib/systemd/system/ffbm.target.wants/systemd-ask-password-wall.path
   if [ "${FACTORY}" == "1" ]; then
       install -m 0750 ${WORKDIR}/mount-factory-data \
                    -D ${D}${sysconfdir}/initscripts/mount-data
   else
   install -m 0750 ${WORKDIR}/mount-data \
                -D ${D}${sysconfdir}/initscripts/mount-data
   fi
   install -d ${D}${systemd_unitdir}/system/
   install -m 0644 ${WORKDIR}/mount-data.service \
                -D ${D}${systemd_unitdir}/system/mount-data.service
   install -d ${D}${systemd_unitdir}/system/local-fs.target.requires/
   ln -sf ${systemd_unitdir}/system/mount-data.service \
               ${D}${systemd_unitdir}/system/local-fs.target.requires/mount-data.service

   install -m 0750 ${WORKDIR}/setup_localtime_link \
                -D ${D}${sysconfdir}/initscripts/setup_localtime_link
   install -m 0644 ${WORKDIR}/setup_localtime_link.service \
                -D ${D}${sysconfdir}/systemd/system/setup_localtime_link.service
   ln -sf ${sysconfdir}/systemd/system/setup_localtime_link.service \
               ${D}${sysconfdir}/systemd/system/multi-user.target.wants/setup_localtime_link.service
}

FILES_${PN} += "/etc/initscripts"
