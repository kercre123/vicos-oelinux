#!/usr/bin/env bash

set -e
set -u

BUILD_ETC=poky/build/tmp-glibc/work/apq8009_robot-oe-linux-gnueabi/machine-robot-image/1.0-r0/rootfs/etc

BASE=`cat ${BUILD_ETC}/os-version-base`
CODE=`cat ${BUILD_ETC}/os-version-code`
TAGNAME="${BASE}.${CODE}"

# Check to see if the victor submodule pointer has been modified
VICTOR_DIR=anki/victor
ANKI_VICTOR_STATUS=`git status --porcelain $VICTOR_DIR | awk '{print $1;}'`

if [ "$ANKI_VICTOR_STATUS" = "M" ]; then
    pushd $VICTOR_DIR
    # Get the SHA for the Victor checkout
    VICTOR_SHA=`git log -1 --format=%h`
    popd

    # Prepare a commit message for updating the victor submodule pointer
    echo "[$VICTOR_DIR $VICTOR_SHA] Build $TAGNAME" > msg.txt
    echo "" >> msg.txt
    git submodule summary >> msg.txt

    # Make a commit for updating the victor submodule pointer
    git add $VICTOR_DIR
    git commit -F msg.txt
    rm msg.txt
fi

# Create a tag and push it to the remote
git tag ${TAGNAME} HEAD

git push origin ${TAGNAME}:refs/tags/${TAGNAME}
