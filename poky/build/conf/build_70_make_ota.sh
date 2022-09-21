#!/bin/bash

set -e

[ -z ${OTA_MANIFEST_SIGNING_KEY+y} ] && echo "OTA_MANIFEST_SIGNING_KEY not defined" && exit 1

while getopts :v:b: flag
do
    case "${flag}" in
	b) BRANCH=${OPTARG};;
	v) BUILD_VARIANT=${OPTARG};;
    esac
    
done

case "${BRANCH}" in
    master)
	BRANCH_SUBFOLDER=master
	;;
    rc)
	BRANCH_SUBFOLDER=rc
	;;
    v20)
	BRANCH_SUBFOLDER=v20
	;;
    v20-l)
	BRANCH_SUBFOLDER=v20-l
	;;
    *) echo "BAD OPTION. '-b master' or '-b rc' or '-b v20'" && exit 1;;
esac

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

cd $WS

VICOS_VERSION=`cat $WS/poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/machine-robot-image/1.0-r0/rootfs/etc/os-version`
export IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf/

cd $WS/ota
case "${BUILD_VARIANT}" in
    dev)
	make verify-boot-dev
	make
    ;;
    prod)
	make prodsign
	make verify-boot-prod
	ANKIDEV=0 make
    ;;
    oskr)
	make oskrsign
	make verify-boot-oskr
	make
    ;;
    ep)
	make prodsign
	make verify-boot-prod
	ANKIDEV=0 make
    ;;
    epdev)
	make
    ;;
    *) echo "BAD OPTION. '-v (dev|prod|oskr|ep|epdev)'" && exit 1;;
esac

cd $WS/_build
mkdir -p ~/vicos-builds/${BRANCH_SUBFOLDER}/${BUILD_VARIANT}/
cp vicos-${VICOS_VERSION}.ota ~/vicos-builds/${BRANCH_SUBFOLDER}/${BUILD_VARIANT}/
cp vicos-${VICOS_VERSION}.ota ~/vicos-builds/${BRANCH_SUBFOLDER}/${BUILD_VARIANT}/latest.ota

