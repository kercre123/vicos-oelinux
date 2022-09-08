#!/bin/bash

set -e

while getopts :v:b: flag
do
    case "${flag}" in
	v) BUILD_VARIANT=${OPTARG};;
    esac
    
done

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

cd $WS

VICOS_VERSION=`cat $WS/poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/machine-robot-image/1.0-r0/rootfs/etc/os-version`
#export IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf/

cd $WS/ota
case "${BUILD_VARIANT}" in
    dbg)
	make
    ;;
    dvt)
	make
    ;;
    pvt)
	make prodsign
	make verify-boot-prod
	ANKIDEV=0 make
    ;;
    *) echo "BAD OPTION. '-v (dbg|dvt|pvt)'" && exit 1;;
esac

cd $WS/_build
mkdir -p ~/vicos-fac-builds/${BUILD_VARIANT}/
cp apq8009-robot-boot.img ~/vicos-fac-builds/${BUILD_VARIANT}/vicos-${VICOS_VERSION}-boot.img
cp apq8009-robot-sysfs.img ~/vicos-fac-builds/${BUILD_VARIANT}/vicos-${VICOS_VERSION}-sysfs.img
cp vicos-${VICOS_VERSION}.ota ~/vicos-fac-builds/${BUILD_VARIANT}
