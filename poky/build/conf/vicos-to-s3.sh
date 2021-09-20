#!/bin/bash

IN_DIR=~/vicos-builds/
OUT_DIR=s3://assets.digitaldreamlabs.com/vic/ufxTn3XGcVNK2YrF

export AWS_ACCESS_KEY_ID=AKIAVLI25QX5MKAPR4TE
aws s3 sync ${IN_DIR} ${OUT_DIR}
