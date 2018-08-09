#!/bin/bash
set -e
set -u

SRC_DIR="$(pwd)/../_build/unlock"
if [ $# -eq 1 ]; then
  SRC_DIR="${1}"
fi

if [ ! -d ${SRC_DIR} ]; then
  echo "error: directory containing unlock ota files not found."
  echo "usage: `basename $0` <path/to/ota/files>"
  exit 1
fi

: ${OTA_TOOL_HOME="$(pwd)/../_build/sai-ota-tool"}
: ${OTA_OPTS=""}
: ${DRY_RUN=0}

# install sai-ota-tool if necessary
if [ ! -d $OTA_TOOL_HOME ]; then
  pushd $(pwd)/../_build
  git clone git@github.com:anki/sai-ota-tool.git
  popd
fi

if [ ! -d $OTA_TOOL_HOME/venv ]; then
  pushd $OTA_TOOL_HOME
  PATH=/usr/local/bin:$PATH virtualenv venv
  ./venv/bin/pip install -r requirements.txt
  popd
fi

if [ -z ${AWS_ACCESS_KEY_ID+x} ]; then
  echo "must set AWS_ACCESS_KEY_ID"
  exit 1
fi

if [ -z ${AWS_SECRET_KEY+x} ]; then
  echo "must set AWS_SECRET_KEY"
  exit 1
fi

if [ $DRY_RUN -eq 0 ]; then
  OTA_OPTS="${OTA_OPTS} --up"
fi

for OTA in ${SRC_DIR}/*.ota; do
  echo "Uploading ${OTA}"
  ${OTA_TOOL_HOME}/venv/bin/python ${OTA_TOOL_HOME}/ota.py \
    --env unlock --full --replace ${OTA_OPTS} \
    "${OTA}"
done
