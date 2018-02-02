#!/usr/bin/env sh
set -xe
DIR=build/tmp-glibc/deploy/images/apq8009-robot-robot
mkdir -p $DIR
rsync -avz --delete vmos:le1.0.2/apps_proc/poky/${DIR}/ ${DIR}/
