DESCRIPTION = "Victor Robot daemon"
LICENSE = "Anki-Inc.-Proprietary"
LIC_FILES_CHKSUM = "file://${COREBASE}/meta-qti-bsp/files/anki-licenses/\
Anki-Inc.-Proprietary;md5=4b03b8ffef1b70b13d869dbce43e8f09"

FILESPATH =+ "${WORKSPACE}:"

SRCREV   = "${AUTOREV}"
BUILDSRC = "${S}/_build/vicos/Release"

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

inherit externalsrc
EXTERNALSRC = "${WORKSPACE}/anki/victor"

export SSH_AUTH_SOCK
export ANKI_BUILD_VERSION

# Must inherit qperf if using the USER_BUILD flag
inherit useradd qperf

# You must set USERADD_PACKAGES when you inherit useradd. This
# lists which output packages will include the user/group
# creation code.
USERADD_PACKAGES = "${PN} "

# For standard Android user/group ids (AID) defs see:
# system/core/include/private/android_filesystem_config.h
# We currently use the reserved OEM range (2900-2999)

# Add groups
GROUPADD_PARAM_${PN} =  "  -g 2901 anki"
GROUPADD_PARAM_${PN} += "; -g 2902 robot"
GROUPADD_PARAM_${PN} += "; -g 2903 engine"
GROUPADD_PARAM_${PN} += "; -g 2904 bluetooth"
GROUPADD_PARAM_${PN} += "; -g 2905 ankinet"
GROUPADD_PARAM_${PN} += "; -g 888 cloud"
GROUPADD_PARAM_${PN} += "; -g 2907 camera"
GROUPADD_PARAM_${PN} += "; -g 1000 system"

# VIC-1951: group 3003 already exists as the inet group (AID_NET 3003)
# Since we have ANDROID_PARANOID_NETWORKING enabled in the kernel, non-admin users
# must be in this group in order to create TCP/UDP sockets

# Add users
USERADD_PARAM_${PN} =  "  -u 2901 -g 2901 -s /bin/false anki"
USERADD_PARAM_${PN} += "; -u 2902 -g 2902 -G 2901,1000 -s /bin/false robot"
USERADD_PARAM_${PN} += "; -u 2903 -g 2903 -G 2901,1000 -s /bin/false engine"
USERADD_PARAM_${PN} += "; -u 2904 -g 2904 -G 2901,1000 -s /bin/false bluetooth"
USERADD_PARAM_${PN} += "; -u 2905 -g 2905 -G 2901,1000,3003 -s /bin/false net"
USERADD_PARAM_${PN} += "; -u 888  -g 888  -G 2901,1000,3003 -s /bin/false cloud"
USERADD_PARAM_${PN} += "; -u 1000 -g 1000 -s /bin/false system"

do_package_qa[noexec] = "1"

do_clean_append() {
    dir = bb.data.expand("${S}", d)
    os.chdir(dir)
    os.system('git clean -Xfd')
}

do_compile () {
   cd ${S}
   source ./project/victor/envsetup.sh
   export TOPLEVEL=`gettop`

   if [[ "${USER_BUILD}" == "1" ]]; then
     if [[ "${DEV}" == "1" ]]; then
       if [[ "${BETA}" == "1" ]]; then
         ./project/victor/scripts/victor_build_beta.sh
       else
         ./project/victor/scripts/victor_build_userdev.sh
       fi
     else
       ./project/victor/scripts/victor_build_shipping.sh
     fi
   else
     ./project/victor/scripts/victor_build_release.sh
   fi
}

do_install () {
    ${S}/project/victor/scripts/install.sh ${BUILDSRC} ${D}

    # Remove "other" permission and remove unnecessary exec on everything in /anki
    # BRC: Setting this here is a dirty hack, we should correctly set permissions in a
    # cmake install step.
    chmod -R 644 ${D}/anki
    chmod 755 ${D}/anki
    chmod 755 ${D}/anki/{data,etc,lib}
    chmod -R 755 ${D}/anki/bin
    chmod -R 755 ${D}/anki/lib
    chown -R 2901:2901 ${D}/anki
}

FILES_${PN} += "anki/"
