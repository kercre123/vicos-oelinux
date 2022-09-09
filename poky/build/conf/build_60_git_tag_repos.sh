#!/bin/bash

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

cd $WS

VICOS_VERSION=`cat poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/machine-robot-image/1.0-r0/rootfs/etc/os-version`
echo TAGGING cozmo-${VICOS_VERSION}

git tag cozmo-${VICOS_VERSION}
git push origin --tags

pushd anki/victor
git tag cozmo-${VICOS_VERSION}
git push origin --tags
popd

pushd anki/vector-cloud
git tag cozmo-${VICOS_VERSION}
git push origin --tags
popd
