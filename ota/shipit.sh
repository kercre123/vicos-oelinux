#!/bin/bash

set -u
set -e

SCRIPT_PATH=$(dirname $([ -L $0 ] && echo "$(dirname $0)/$(readlink -n $0)" || echo $0))            
SCRIPT_NAME=`basename ${0}`

function usage()
{
    echo "usage: $SCRIPT_NAME [options] <build-version>"
    echo "  -h                      Print this help message"
    echo "  -v                      Print verbose output"
    echo "  -o OUTPUT_DIR           Download artifacts to OUTPUT_DIR"
    echo "  -u TC_USER              Teamcity username"
}


: ${VERBOSE:=0}
: ${OUTPUT_DIR:="${SCRIPT_PATH}/../_build/tc-artifacts"}
TC_USER=
TC_PASS=

function logv()
{
    if [ $VERBOSE -eq 1 ]; then echo "$1"; fi
}

while getopts ":o:u:vh" opt; do
    case $opt in
        h)
            usage
            exit 1
            ;;
        v)
            VERBOSE=1
            ;;
        o)
            OUTPUT_DIR="${OPTARG}"
            ;;
        u)
            TC_USER="${OPTARG}"
            ;;
    esac
done

# Move past getops args                                                                             
shift $(($OPTIND - 1))

if [ $# -ne 1 ]; then
    usage
    exit 1
fi

BUILD_VERSION="$1"
BUILD_NUMBER_FIELD=${BUILD_VERSION##*.}
BUILD_NUMBER=${BUILD_NUMBER_FIELD//[!0-9]/}
logv "Fetch version $BUILD_VERSION (build number $BUILD_NUMBER)"

if [ -z ${TC_USER} ]; then
    read -r -p "Enter Teamcity username: " TC_USER
fi

if [ -z ${TC_PASS} ]; then
    read -r -s -p "Enter Teamcity password: " TC_PASS
fi

TC_AUTH="${TC_USER}:${TC_PASS}"

ARTIFACTS=(
    apq8009-robot-boot.img.nonsecure
    apq8009-robot-sysfs.ext4
)

mkdir -p "${OUTPUT_DIR}"
pushd ${OUTPUT_DIR}

for IMG in "${ARTIFACTS[@]}"; do
    URL="https://build.ankicore.com/repository/download/Vicos_Release/${BUILD_NUMBER}/${IMG}"
    logv "fetch ${URL}"
    curl \
        -O \
        --user "${TC_AUTH}" \
        "${URL}"
done

popd

logv "prodsign boot.img"
make prodsign \
    IMG_DIR="${OUTPUT_DIR}" \
    UPDATE_VERSION="${BUILD_VERSION}" \
    ANKIDEV=0

logv "make prod OTA image"
read -r -s -p "Enter pass phrase for OTA signing key: " OTA_KEY_PASS
make all \
    IMG_DIR="${OUTPUT_DIR}" \
    OTA_KEY_PASS="${OTA_KEY_PASS}" \
    UPDATE_VERSION="${BUILD_VERSION}" \
    ANKIDEV=0

logv "rename upload file"
ln ../_build/vicos-${BUILD_VERSION}.ota ../_build/${BUILD_VERSION}.ota

logv "OTA manifest dump"
tar -O -xf ../_build/${BUILD_VERSION}.ota manifest.ini

echo "Generated OTA file: ../_build/${BUILD_VERSION}.ota"
