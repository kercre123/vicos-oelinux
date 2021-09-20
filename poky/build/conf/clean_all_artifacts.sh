#!/bin/bash
#
# Export settings so we get the build version in the app.
#

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)

echo CLEANING YOCTO BUILD DIRS...
cd ${WS}/poky/build
sudo rm -fr cache/ sstate-cache/ tmp-glibc/

echo CLEANING VICTOR BUILD DIRS...
cd ${WS}/anki/victor
rm -fr _build generated

echo CLEANING VIC-CLOUD BUILD DIR...
cd ${WS}/anki/vector-cloud
sudo rm -fr build
