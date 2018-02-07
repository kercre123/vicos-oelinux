# set_bb_env.sh
# Define macros for build targets.
# Generate bblayers.conf from get_bblayers.py.
# Some convenience macros are defined to save some typing.
# Set the build environement
if [[ ! $(readlink $(which sh)) =~ bash ]]
then
  echo ""
  echo "### ERROR: Please Change your /bin/sh symlink to point to bash. ### "
  echo ""
  echo "### sudo ln -sf /bin/bash /bin/sh ### "
  echo ""
  return 1
fi

# The SHELL variable also needs to be set to /bin/bash otherwise the build
# will fail, use chsh to change it to bash.
if [[ ! $SHELL =~ bash ]]
then
  echo ""
  echo "### ERROR: Please Change your shell to bash using chsh. ### "
  echo ""
  echo "### Make sure that the SHELL variable points to /bin/bash ### "
  echo ""
  return 1
fi

umask 022
unset DISTRO MACHINE PRODUCT VARIANT

# OE doesn't want a set-gid directory for its tmpdir
BT="./build/tmp-glibc"
if [ ! -d ${BT} ]
then
  mkdir -m u=rwx,g=rx,g-s,o=  ${BT}
elif [ -g ${BT} ]
then
  chmod -R g-s ${BT}
fi
unset BT

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

# Add a few helpful shortcuts
# Go to root of workspace
alias croot='cd $WS'

# Go to the directory from where you can kick off the build(workspace/poky/build)
# from wherever you are
alias gobuilddir='CUR_DIR=`pwd` && cd $WS/poky/build'

# Go back to the directory you were working in before you ran gobuild
alias goback='cd $CUR_DIR'

#Build recipe X for target Y
function build_x_for_y() { gobuilddir && MACHINE=$2 rebake $1 && goback; }

#Go to OUT directory
alias goout='croot && cd poky/build/tmp-glibc/deploy/images/$MACHINE'


# Dynamically generate our bblayers.conf since we effectively can't whitelist
# BBLAYERS (by OE-Core class policy...Bitbake understands it...) to support
# dynamic workspace layer functionality.
python $scriptdir/get_bblayers.py ${WS}/poky \"meta*\" > $scriptdir/bblayers.conf

# Convienence functions provided for the QuIC provided OE Linux distro.

# 9650 commands
function build-9650-perf-image() {
  unset_bb_env
  export MACHINE=mdm9650
  export PRODUCT=base
  export DISTRO=mdm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-9650-image() {
  unset_bb_env
  export MACHINE=mdm9650
  export PRODUCT=base
  cdbitbake machine-image
}

function build-9650-2k-image() {
  unset_bb_env
  export MACHINE=mdm9650-2k
  export PRODUCT=base
  cdbitbake machine-image
}

function build-9650-2k-perf-image() {
  unset_bb_env
  export MACHINE=mdm9650-2k
  export PRODUCT=base
  export DISTRO=mdm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

build-all-9650-images() {
  build-9650-image
  build-9650-2k-image
  build-9650-perf-image
  build-9650-2k-perf-image
}

function build-9650-psm-image() {
  unset_bb_env
  export MACHINE=mdm9650
  export PRODUCT=psm
  cdbitbake machine-psm-image
}

function build-9650-psm-perf-image() {
  unset_bb_env
  export MACHINE=mdm9650
  export DISTRO=msm-perf
  export VARIANT=perf
  export PRODUCT=psm
  cdbitbake machine-psm-image
}

build-all-9650-psm-images() {
  build-9650-psm-image
  build-9650-psm-perf-image
}

# 8009 commands
function build-8009-perf-image() {
  unset_bb_env
  export MACHINE=apq8009
  export DISTRO=msm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-8009-user-image() {
  unset_bb_env
  export MACHINE=apq8009
  export DISTRO=msm-user
  export VARIANT=user
  cdbitbake machine-image
}


function build-8009-image() {
  unset_bb_env
  export MACHINE=apq8009
  export PRODUCT=base
  cdbitbake machine-image
}

build-all-8009-images() {
  build-8009-image
  build-8009-perf-image
}

function build-8009-robot-image() {
  unset_bb_env
  export MACHINE=apq8009-robot
  export PRODUCT=robot
  cdbitbake machine-robot-image
}

function build-8009-robot-perf-image() {
  unset_bb_env
  export MACHINE=apq8009-robot
  export DISTRO=msm-perf
  export VARIANT=perf
  export PRODUCT=robot
  cdbitbake machine-robot-image
}

function build-8009-robot-dummy-image() {
  unset_bb_env
  export MACHINE=apq8009-robot
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8009-robot-rome-image() {
  unset_bb_env
  export MACHINE=apq8009-robot
  export PRODUCT=robot-rome
  cdbitbake machine-robot-image
}

function build-8009-robot-rome-perf-image() {
  unset_bb_env
  export MACHINE=apq8009-robot
  export DISTRO=msm-perf
  export VARIANT=perf
  export PRODUCT=robot-rome
  cdbitbake machine-robot-image
}

build-all-8009-robot-images() {
  build-8009-robot-image
  build-8009-robot-perf-image
  build-8009-robot-rome-image
  build-8009-robot-rome-perf-image
}

build-all-8009-drone-images() {
  build-8009-drone-image
  build-8009-drone-perf-image
}

# 8017 commands
function build-8017-perf-image() {
  unset_bb_env
  export MACHINE=apq8017
  export DISTRO=msm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-8017-image() {
  unset_bb_env
  export MACHINE=apq8017
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8017-user-image() {
  unset_bb_env
  export MACHINE=apq8017
  export DISTRO=msm-user
  export VARIANT=user
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8017-qsap-image() {
  unset_bb_env
  export MACHINE=apq8017
  export PRODUCT=qsap
  cdbitbake machine-qsap-image
}

function build-8017-qsap-perf-image() {
  unset_bb_env
  export MACHINE=apq8017
  export DISTRO=msm-perf
  export VARIANT=perf
  export PRODUCT=qsap
  cdbitbake machine-qsap-image
}

function build-8017-qsap-user-image() {
  unset_bb_env
  export MACHINE=apq8017
  export DISTRO=msm-user
  export VARIANT=user
  export PRODUCT=qsap
  cdbitbake machine-qsap-image
}

build-all-8017-images() {
  build-8017-image
  build-8017-perf-image
  build-8017-user-image
}

build-all-8017-qsap-images() {
  build-8017-qsap-image
  build-8017-qsap-perf-image
}

# 9607 commands
function build-9607-perf-image() {
  unset_bb_env
  export MACHINE=mdm9607
  export DISTRO=mdm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-9607-psm-image() {
  unset_bb_env
  export MACHINE=mdm9607
  export PRODUCT=psm
  cdbitbake machine-psm-image
}

function build-9607-image() {
  unset_bb_env
  export MACHINE=mdm9607
  export PRODUCT=base
  cdbitbake machine-image
}

build-all-9607-images() {
  build-9607-image
  build-9607-perf-image
#  build-9607-psm-image
}

# 8909w commands
function build-8909w-image() {
  unset_bb_env
  export MACHINE=msm8909w
  export PRODUCT=base
  cdbitbake machine-image
}

# 8053 commands
function build-8053-image() {
  unset_bb_env
  export MACHINE=apq8053
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8053-perf-image() {
  unset_bb_env
  export MACHINE=apq8053
  export DISTRO=msm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

build-all-8053-images() {
  build-8053-image
  build-8053-perf-image
}

# 8053-32 commands
function build-8053-32-image() {
  unset_bb_env
  export MACHINE=apq8053-32
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8053-32-perf-image() {
  unset_bb_env
  export MACHINE=apq8053-32
  export DISTRO=msm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-8053-32-minimal-image() {
  unset_bb_env
  export MACHINE=apq8053-32
  export PRODUCT=base
  export VARIANT=minimal
  cdbitbake machine-minimal-image
}

function build-8053-32-perf-minimal-image() {
  unset_bb_env
  export MACHINE=apq8053-32
  export DISTRO=msm-perf
  export VARIANT=perf-minimal
  cdbitbake machine-minimal-image
}

build-all-8053-32-images() {
  build-8053-32-minimal-image
  build-8053-32-image
  build-8053-32-perf-minimal-image
  build-8053-32-perf-image
}

# 8096 commands
function build-8096-image() {
  unset_bb_env
  export MACHINE=apq8096
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8096-perf-image() {
  unset_bb_env
  export MACHINE=apq8096
  export DISTRO=msm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-8096-drone-image() {
  unset_bb_env
  export MACHINE=apq8096
  export PRODUCT=drone
  cdbitbake machine-drone-image
}

function build-8096-drone-perf-image() {
  unset_bb_env
  export MACHINE=apq8096
  export PRODUCT=drone
  export DISTRO=msm-perf
  cdbitbake machine-drone-image
}

build-all-8096-images() {
  build-8096-image
  build-8096-perf-image
}

build-all-8096-drone-images() {
  build-8096-drone-image
  build-8096-drone-perf-image
}

# sdx20 commands
function build-sdx20-perf-image() {
  unset_bb_env
  export MACHINE=sdx20
  export DISTRO=mdm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

function build-sdx20-image() {
  unset_bb_env
  export MACHINE=sdx20
  export PRODUCT=base
  cdbitbake machine-image
}

build-all-sdx20-images() {
  build-sdx20-image
  build-sdx20-perf-image
}

# 8098 commands
function build-8098-image() {
  unset_bb_env
  export MACHINE=apq8098
  export PRODUCT=base
  cdbitbake machine-image
}

function build-8098-perf-image() {
  unset_bb_env
  export MACHINE=apq8098
  export DISTRO=msm-perf
  export VARIANT=perf
  cdbitbake machine-image
}

build-all-8098-images() {
  build-8098-image
  build-8098-perf-image
}

# Utility commands
buildclean() {
  set -x
  cd ${WS}/poky/build

  rm -rf bitbake.lock pseudodone sstate-cache tmp-glibc/* cache && cd - || cd -
  set +x
}

# Lists only those build commands that are:
#   * prefixed with function keyword
#   * name starts with build-

list-build-commands()
{
    echo
    echo "Convenience commands for building images:"
    local script_file="$WS/poky/build/conf/set_bb_env.sh"

    while IFS= read line; do
        if echo $line | grep -q "^function[[:blank:]][[:blank:]]*build-"; then
            local delim_string=$(echo $line | cut -d'(' -f1)
            echo "   $(echo $delim_string|awk -F "[[:blank:]]*" '{print $2}')"
        fi
    done < $script_file

    echo
    echo "Use 'list-build-commands' to see this list again."
    echo
}

cdbitbake() {
  local ret=0
  cd ${WS}/poky/build
  bitbake $@ && cd - || ret=$? && cd -
  return $ret
}

rebake() {
  cdbitbake -c cleanall $@ && \
  cdbitbake $@
}

unset_bb_env() {
  unset DISTRO MACHINE PRODUCT VARIANT
}

# Find build templates from qti meta layer.
export TEMPLATECONF="meta-qti-bsp/conf"

# Yocto/OE-core works a bit differently than OE-classic so we're
# going to source the OE build environment setup script they provided.
# This will dump the user in ${WS}/yocto/build, ready to run the 
# convienence function or straight up bitbake commands.
. ${WS}/poky/oe-init-build-env

# Let bitbake use the following env-vars as if they were pre-set bitbake ones.
# (BBLAYERS is explicitly blocked from this within OE-Core itself, though...)
# oe-init-build-env calls oe-buildenv-internal which sets
# BB_ENV_EXTRAWHITE, append our vars to the list
export BB_ENV_EXTRAWHITE="${BB_ENV_EXTRAWHITE} DL_DIR PRODUCT VARIANT"

list-build-commands
