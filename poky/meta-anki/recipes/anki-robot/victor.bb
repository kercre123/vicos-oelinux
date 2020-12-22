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

GID_ANKI      = '2901'
GID_ROBOT     = '2902'
GID_ENGINE    = '2903'
GID_BLUETOOTH = '2904'
GID_ANKINET   = '2905'
GID_CLOUD     = '888'
GID_CAMERA    = '2907'
GID_SYSTEM    = '1000'

# Add groups
GROUPADD_PARAM_${PN} =  "  -g ${GID_ANKI} anki"
GROUPADD_PARAM_${PN} += "; -g ${GID_ROBOT} robot"
GROUPADD_PARAM_${PN} += "; -g ${GID_ENGINE} engine"
GROUPADD_PARAM_${PN} += "; -g ${GID_BLUETOOTH} bluetooth"
GROUPADD_PARAM_${PN} += "; -g ${GID_ANKINET} ankinet"
GROUPADD_PARAM_${PN} += "; -g ${GID_CLOUD} cloud"
GROUPADD_PARAM_${PN} += "; -g ${GID_CAMERA} camera"
GROUPADD_PARAM_${PN} += "; -g ${GID_SYSTEM} system"

# VIC-1951: group 3003 already exists as the inet group (AID_NET 3003)
# Since we have ANDROID_PARANOID_NETWORKING enabled in the kernel, non-admin users
# must be in this group in order to create TCP/UDP sockets

AID_NET       = '3003'
UID_ANKI      = "${GID_ANKI}"
UID_ROBOT     = "${GID_ROBOT}"
UID_ENGINE    = "${GID_ENGINE}"
UID_BLUETOOTH = "${GID_BLUETOOTH}"
UID_NET       = "${GID_ANKINET}"
UID_CLOUD     = "${GID_CLOUD}"
UID_SYSTEM    = "${GID_SYSTEM}"
# Add users
USERADD_PARAM_${PN} =  "  -u ${UID_ANKI} -g ${GID_ANKI} -s /bin/false anki"
USERADD_PARAM_${PN} += "; -u ${UID_ROBOT} -g ${GID_ROBOT} -G ${GID_ANKI},${GID_SYSTEM} -s /bin/false robot"
USERADD_PARAM_${PN} += "; -u ${UID_ENGINE} -g ${GID_ENGINE} -G ${GID_ANKI},${GID_SYSTEM},${AID_NET},${GID_BLUETOOTH},${GID_CAMERA} -s /bin/false engine"
USERADD_PARAM_${PN} += "; -u ${UID_BLUETOOTH} -g ${GID_BLUETOOTH} -G ${GID_ANKI},${GID_SYSTEM} -s /bin/false bluetooth"
USERADD_PARAM_${PN} += "; -u ${UID_NET} -g ${GID_ANKINET} -G ${GID_ANKI},${GID_BLUETOOTH},${GID_SYSTEM},${AID_NET} -s /bin/false net"
USERADD_PARAM_${PN} += "; -u ${UID_CLOUD} -g ${GID_CLOUD} -G ${GID_ANKI},${GID_SYSTEM},${AID_NET} -s /bin/false cloud"
USERADD_PARAM_${PN} += "; -u ${UID_SYSTEM} -g ${GID_SYSTEM} -s /bin/false system"

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

  if [[ "${ANKI_AMAZON_ENDPOINTS_ENABLED}" == "1" ]]; then
    if [[ "${USER_BUILD}" == "1" ]]; then
      if [[ "${DEV}" == "1" ]]; then
        if [[ "${BETA}" == "1" ]]; then
	        ./project/victor/scripts/victor_build_alexa_beta.sh
        elif [[ "${ANKI_RESOURCE_ESCAPEPOD}" == "1" ]]; then
          ./project/victor/scripts/victor_build_escape_pod_userdev.sh
        else
	        ./project/victor/scripts/victor_build_alexa_userdev.sh
        fi
      else
        ./project/victor/scripts/victor_build_alexa_shipping.sh
      fi
    elif [[ "${ANKI_RESOURCE_ESCAPEPOD}" == "1" ]]; then
      ./project/victor/scripts/victor_build_escape_pod_userdev.sh
    else
      ./project/victor/scripts/victor_build_alexa_release.sh
    fi
  else
    if [[ "${USER_BUILD}" == "1" ]]; then
      if [[ "${DEV}" == "1" ]]; then
        if [[ "${BETA}" == "1" ]]; then
	        ./project/victor/scripts/victor_build_beta.sh
        elif [[ "${ANKI_RESOURCE_ESCAPEPOD}" == "1" ]]; then
          ./project/victor/scripts/victor_build_escape_pod_userdev.sh
        else
	        ./project/victor/scripts/victor_build_userdev.sh
        fi
      elif [[ "${ANKI_RESOURCE_ESCAPEPOD}" == "1" ]]; then
        ./project/victor/scripts/victor_build_escape_pod_release.sh
      else
        ./project/victor/scripts/victor_build_shipping.sh
      fi
    else
      if [[ "${OSKR}" == "1" ]]; then
        ./project/victor/scripts/victor_build_oskr.sh
      elif [[ "${ANKI_RESOURCE_ESCAPEPOD}" == "1" ]]; then
        ./project/victor/scripts/victor_build_escape_pod_release.sh
      else
        ./project/victor/scripts/victor_build_release.sh
      fi
    fi
  fi
}

do_install () {
  ${S}/project/victor/scripts/install.sh ${BUILDSRC} ${D}
}

do_generate_victor_canned_fs_config () {
  CANNED_FS_CONFIG_PATH="${DEPLOY_DIR_IMAGE}/victor_canned_fs_config"
  cat > ${CANNED_FS_CONFIG_PATH} <<EOF
anki                              ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/diagnostics-logger       ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/displayFaultCode         ${UID_ENGINE} ${GID_ANKI} 0550
anki/bin/update-engine            ${UID_NET}    ${GID_ANKI} 0550
anki/bin/vic-anim                 ${UID_ENGINE} ${GID_ANKI} 0500
anki/bin/vic-bootAnim             ${UID_ENGINE} ${GID_ANKI} 0550
anki/bin/vic-cloud                ${UID_CLOUD}  ${GID_ANKI} 0550
anki/bin/vic-crashuploader-init   ${UID_NET}    ${GID_ANKI} 0550
anki/bin/vic-crashuploader        ${UID_NET}    ${GID_ANKI} 0550
anki/bin/vic-dasmgr               ${UID_NET}    ${GID_ANKI} 0500
anki/bin/vic-engine               ${UID_ENGINE} ${GID_ANKI} 0500
anki/bin/vic-faultCodeDisplay     ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-gateway              ${UID_NET}    ${GID_ANKI} 0500
anki/bin/vic-getprocessstatus.sh  ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-init.sh              ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-log-cat              ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-log-event            ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-log-forward          ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-log-upload           ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-log-uploader         ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-logmgr-upload        ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-on-exit              ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-powerstatus.sh       ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-robot                ${UID_ROBOT}  ${GID_ANKI} 0550
anki/bin/vic-runcrashtests.sh     ${UID_ANKI}   ${GID_ANKI} 0550
anki/bin/vic-switchboard          ${UID_NET}    ${GID_ANKI} 0500
anki/bin/vic-webserver            ${UID_ANKI}   ${GID_ANKI} 0500

EOF
  # Directories should be readable and searchable by the anki group
  find ${D}/anki -type d \
    -printf "anki/%P  ${UID_ANKI} ${GID_ANKI} 0550\n" >> ${CANNED_FS_CONFIG_PATH}

  # Files under data, etc, and lib should be readable by the anki group
  for i in data etc lib
  do
    find ${D}/anki/$i -type f \
      -printf "anki/$i/%P  ${UID_ANKI} ${GID_ANKI} 0440\n" >> ${CANNED_FS_CONFIG_PATH}
  done
}

addtask generate_victor_canned_fs_config after do_install before do_package

#
# Add custom task to copy OS symbol files into victor build tree.
#
inherit anki-symbol-files

do_anki_symbol_import () {
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
DEPENDS += "curl"
DEPENDS += "glib-2.0"
DEPENDS += "glibc"
DEPENDS += "libgcc"
DEPENDS += "liblog"
DEPENDS += "libpcre"
DEPENDS += "libunwind"
DEPENDS += "liburcu"
DEPENDS += "linux-quic"
DEPENDS += "lttng-ust"
DEPENDS += "sqlite3"
DEPENDS += "zlib"

do_anki_symbol_import[deptask] = "do_anki_symbol_export"

#
# Declare files produced by this package
#

FILES_${PN} += "anki/"
