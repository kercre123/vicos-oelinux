#!/bin/bash

set -e

#
# Export settings so we get the build version in the app.
#

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
CONF_DIR=~/vicos-builds
CONF_FILE=anki_build_version.txt
REMOTE_URL=http://assets.digitaldreamlabs.com/vic/ufxTn3XGcVNK2YrF/
OUT_DIR=s3://assets.digitaldreamlabs.com/vic/ufxTn3XGcVNK2YrF/

mkdir -p ${CONF_DIR}
wget -P ${CONF_DIR} ${REMOTE_URL}${CONF_FILE}

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

export AWS_ACCESS_KEY_ID=AKIAVLI25QX5MKAPR4TE
aws s3 mv ${BUILD_VERSION_FILE} ${OUT_DIR}

echo Updated build version to $NEXT_BUILD_VERSION ...
