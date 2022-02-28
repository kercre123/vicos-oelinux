#!/bin/bash
set -e

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"

# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

cd $WS/poky

source build/conf/set_bb_env.sh

cd $WS/poky

source build/conf/set_anki_build_version.sh

cd $WS/poky/build

conf/build_40_clean_all_artifacts.sh

while getopts v: flag
do
    case "${flag}" in
	v) BUILD_VARIANT=${OPTARG};;
    esac
done

echo BUILDING ${BUILD_VARIANT} BUILD NUMBER ${ANKI_BUILD_VERSION}...

case "${BUILD_VARIANT}" in
    dev) build-victor-robot-perf-image;;
    fac) build-victor-robot-factory-image;;
    *) echo "BAD OPTION. Use '-v (dev|fac)'" && exit 1;;
esac
