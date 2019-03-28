#!/bin/bash

set -e

SCRIPT_NAME=$(basename $0)

# defaults
VERBOSE=0
BUILD_COMMAND=("build-victor-robot-perf-image")

function usage() {
    echo "$SCRIPT_NAME [OPTIONS] <bitbake build command/function> (default: ${BUILD_COMMAND[*]})"
    echo "  -h          print this message"
    echo "  -v          print verbose output"
}

while getopts ":hv" opt; do
    case $opt in
        h)
            usage
            exit 1
            ;;
        v)
            VERBOSE=1
            ;;
        :)
            echo "Options -${OPTARG} requires an argument." >&2
            usage
            exit 1
            ;;
    esac
done

if [ $VERBOSE -eq 1 ]; then
    set -x
fi

# Move past getops args
shift $(($OPTIND - 1))

if [ $# -gt 0 ]; then
    BUILD_COMMAND=$*
fi

echo "Build starting at `date`"

SCRIPT_PATH=$(dirname $([ -L $0 ] && echo "$(dirname $0)/$(readlink -n $0)" || echo $0))
TOPLEVEL=$(cd "${SCRIPT_PATH}/../.." && pwd)

# remove any existing artifacts
pushd $TOPLEVEL/poky

pushd build
# known build artifact dirs
rm -rf bitbake.lock cache downloads  sstate-cache  tmp-glibc

# remove generated files under build/conf
git clean -xddf -- ./conf
popd

# Disable check for unset variables (it will make the bitbake scripts exit)
set +u
source build/conf/set_bb_env.sh

: ${ANKI_BUILD_VERSION:=0}
export ANKI_BUILD_VERSION
export BB_ENV_EXTRAWHITE="$BB_ENV_EXTRAWHITE ANKI_BUILD_VERSION"

${BUILD_COMMAND[*]}

popd

# Re-enable check for unset variables
set -u

echo "Build finished at `date`"
