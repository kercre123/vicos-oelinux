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

# Prevent yocto from splitting out debug files for this recipe
INHIBIT_PACKAGE_DEBUG_SPLIT = '1'
# Victor's CMake build process already strips libs & exes, don't strip again.
INHIBIT_PACKAGE_STRIP = '1'

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
USERADD_PARAM_${PN} += "; -u 2903 -g 2903 -G 2901,1000,3003,2904,2907 -s /bin/false engine"
USERADD_PARAM_${PN} += "; -u 2904 -g 2904 -G 2901,1000 -s /bin/false bluetooth"
USERADD_PARAM_${PN} += "; -u 2905 -g 2905 -G 2901,2904,1000,3003 -s /bin/false net"
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

  #
  # VIC-6026: Publish OS debug symbols into victor lib
  #
  # These files will be picked up as artifacts of the build and
  # sent to backtrace for crash reporting.
  #
  SRCDIR="${WORKSPACE}/poky/build/tmp-glibc/work/armv7a-vfp-neon-oe-linux-gnueabi/glibc/2.22-r0/package/lib/.debug"
  DSTDIR="${BUILDSRC}/lib"
  LIBS="ld-2.22.so libc-2.22.so libdl-2.22.so libm-2.22.so libpthread-2.22.so libresolv-2.22.so librt-2.22.so"
  for lib in ${LIBS} ; do
    install ${SRCDIR}/${lib} ${DSTDIR}/${lib}.full
  done

}

do_install () {
  ${S}/project/victor/scripts/install.sh ${BUILDSRC} ${D}
}

#
# Add custom task to copy OS symbol files into victor build tree.
#
inherit anki-symbol-files

do_anki_symbol_import() {
  # Copy OS symbol files into victor build tree
  pushd ${ANKI_LIB_SYMBOL_DIR}
  for f in * ; do
    install ${f} ${BUILDSRC}/lib/${f}.full
  done
  popd
}

addtask anki_symbol_import after do_install before do_package
  
#
# Declare task dependency to insure that export steps run before import step
#
DEPENDS += "glibc"
DEPENDS += "liblog"
DEPENDS += "libunwind"
DEPENDS += "liburcu"
DEPENDS += "lttng-ust"
DEPENDS += "curl"

do_anki_symbol_import[deptask] = "do_anki_symbol_export"

#
# Declare files produced by this package
#

FILES_${PN} += "anki/"
