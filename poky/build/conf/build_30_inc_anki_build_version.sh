#!/bin/bash

#
# Export settings so we get the build version in the app.
#

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
CONF_DIR=~/vicos-fac-builds
mkdir -p ${CONF_DIR}
BUILD_VERSION_FILE=$CONF_DIR/anki_build_version.txt

if [[ -f "$BUILD_VERSION_FILE" ]];then
    echo "FOUND $BUILD_VERSION_FILE"
    CURRENT_BUILD_VERSION=$(cat $BUILD_VERSION_FILE)
else
    echo "DIDN'T FIND $BUILD_VERSION_FILE"
    CURRENT_BUILD_VERSION=6000
fi

NEXT_BUILD_VERSION=$(expr "$CURRENT_BUILD_VERSION" + 1)
echo $NEXT_BUILD_VERSION > $BUILD_VERSION_FILE
echo Updated build version to $NEXT_BUILD_VERSION ...
