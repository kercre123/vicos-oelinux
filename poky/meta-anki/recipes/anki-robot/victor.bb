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
GROUPADD_PARAM_${PN} += "; -g 2906 cloud"
GROUPADD_PARAM_${PN} += "; -g 2907 camera"

# Add users
USERADD_PARAM_${PN} =  "  -u 2901 -g 2901 -s /bin/false anki"
USERADD_PARAM_${PN} += "; -u 2902 -g 2902 -G 2901 -s /bin/false robot"
USERADD_PARAM_${PN} += "; -u 2903 -g 2903 -G 2901 -s /bin/false engine"
USERADD_PARAM_${PN} += "; -u 2904 -g 2904 -G 2901 -s /bin/false bluetooth"
USERADD_PARAM_${PN} += "; -u 2905 -g 2905 -G 2901 -s /bin/false net"
USERADD_PARAM_${PN} += "; -u 2906 -g 2906 -G 2901 -s /bin/false cloud"

do_package_qa[noexec] = "1"

do_compile () {
   cd ${S}
   source ./project/victor/envsetup.sh
   export TOPLEVEL=`gettop`

   if [ -x ./project/victor/scripts/victor_build_shipping.sh ]; then

       if [[ ${USER_BUILD} == "1" ]]; then
	   ./project/victor/scripts/victor_build_shipping.sh
       else
	   ./project/victor/scripts/victor_build_release.sh
       fi

   else
       BUILD_FLAVOR_FLAGS=""
       if [[ ${USER_BUILD} == "1" ]]; then
	   BUILD_FLAVOR_FLAGS="-DANKI_NO_WEBSERVER_ENABLED=1 \
		-DANKI_DEV_CHEATS=0 \
		-DANKI_PROFILING_ENABLED=0 \
		-DREMOTE_CONSOLE_ENABLED=0 \
		-a -DAUDIO_RELEASE=ON"
       fi
       ./project/victor/build-victor.sh -p vicos -c Release -v $BUILD_FLAVOR_FLAGS
   fi
}

do_install () {
    ${S}/project/victor/scripts/install.sh ${BUILDSRC} ${D}

    # Remove "other" permission and remove unnecessary exec on everything in /anki
    # BRC: Setting this here is a dirty hack, we should correctly set permissions in a
    # cmake install step.
    chmod -R 640 ${D}/anki
    chmod 750 ${D}/anki
    chmod 750 ${D}/anki/{data,etc,lib}
    chmod -R 750 ${D}/anki/bin
}

FILES_${PN} += "anki/"
