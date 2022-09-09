#!/bin/bash

OUT_DIR=~/cozmo-builds/
IN_DIR=s3://assets.digitaldreamlabs.com/cozmo/6TQMDcxAqX8yfZcz

export AWS_ACCESS_KEY_ID=AKIAVLI25QX5MKAPR4TE
aws s3 sync ${IN_DIR} ${OUT_DIR}
