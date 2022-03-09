#!/bin/bash

set -e

[ -z ${AWS_ACCESS_KEY_ID+y} ] && echo "AWS_ACCESS_KEY_ID not defined" && exit 1
[ -z ${AWS_SECRET_ACCESS_KEY+y} ] && echo "AWS_SECRET_ACCESS_KEY not defined" && exit 1
#
# Export settings so we get the build version in the app.
#

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
CONF_DIR=~/vicos-builds
CONF_FILE=anki_build_version.txt
OUT_DIR=s3://assets.digitaldreamlabs.com/vic/ufxTn3XGcVNK2YrF/
BUILD_VERSION_FILE=${CONF_DIR}/${CONF_FILE}

mkdir -p ${CONF_DIR}
aws s3 cp  ${OUT_DIR}${CONF_FILE} ${BUILD_VERSION_FILE}

if [[ -f "$BUILD_VERSION_FILE" ]];then
    echo "FOUND $BUILD_VERSION_FILE"
    CURRENT_BUILD_VERSION=$(cat $BUILD_VERSION_FILE)
else
    echo "DIDN'T FIND $BUILD_VERSION_FILE"
    CURRENT_BUILD_VERSION=6000
fi

NEXT_BUILD_VERSION=$(expr "$CURRENT_BUILD_VERSION" + 1)
echo $NEXT_BUILD_VERSION > $BUILD_VERSION_FILE

aws s3 cp ${BUILD_VERSION_FILE} ${OUT_DIR}

echo Updated build version to $NEXT_BUILD_VERSION ...
