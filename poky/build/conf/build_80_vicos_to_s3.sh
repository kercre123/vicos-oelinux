#!/bin/bash

[ -z ${AWS_ACCESS_KEY_ID+y} ] && echo "AWS_ACCESS_KEY_ID not defined" && exit 1
[ -z ${AWS_SECRET_ACCESS_KEY+y} ] && echo "AWS_SECRET_ACCESS_KEY not defined" && exit 1

IN_DIR=~/cozmo-builds/
OUT_DIR=s3://assets.digitaldreamlabs.com/cozmo/6TQMDcxAqX8yfZcz

aws s3 sync ${IN_DIR} ${OUT_DIR}
