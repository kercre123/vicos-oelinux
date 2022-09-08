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
    dvt)
	make
    ;;
    pvt)
	make prodsign
	make verify-boot-prod
	ANKIDEV=0 make
    ;;
    *) echo "BAD OPTION. '-v (dvt|pvt)'" && exit 1;;
esac

cd $WS/_build
mkdir -p ~/cozmo-fac-builds/${BUILD_VARIANT}/
cp apq8009-robot-boot.img ~/cozmo-fac-builds/${BUILD_VARIANT}/cozmo-${VICOS_VERSION}-boot.img
cp apq8009-robot-sysfs.img ~/cozmo-fac-builds/${BUILD_VARIANT}/cozmo-${VICOS_VERSION}-sysfs.img
cp cozmo-${VICOS_VERSION}.ota ~/cozmo-fac-builds/${BUILD_VARIANT}
