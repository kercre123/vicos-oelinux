#!/usr/bin/env bash

set -e
set -u

VICTOR_DIR=anki/victor
# Use the version number, like 0.10.1304d, as the tag name
TAGNAME=`cat $VICTOR_DIR/_build/vicos/Release/etc/version`

# Check to see if the victor submodule pointer has been modified
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
