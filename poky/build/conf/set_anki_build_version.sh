#
# Export settings so we get the build version in the app.
#

# Find where the global conf directory is...
scriptdir="$(dirname "${BASH_SOURCE}")"
# Find where the workspace is...
WS=$(readlink -f $scriptdir/../../..)
CONF_DIR=$WS/poky/build/conf
BUILD_VERSION_FILE=$CONF_DIR/anki_build_version.txt

if [[ -f "$BUILD_VERSION_FILE" ]];then
    CURRENT_BUILD_VERSION=$(cat $BUILD_VERSION_FILE)
else
    CURRENT_BUILD_VERSION=6000
fi

export ANKI_BUILD_VERSION=$CURRENT_BUILD_VERSION
export BB_ENV_EXTRAWHITE="$BB_ENV_EXTRAWHITE ANKI_BUILD_VERSION"

echo Set anki build version to $CURRENT_BUILD_VERSION ...
