#!/bin/bash

git clean -xffd
cp -rf poky opensource/
rm -rf opensource/poky/*-prop
rm -rf opensource/poky/meta-anki

cp -rf kernel opensource/
cp -rf qcom-opensource opensource/
cp -rf system opensource/
cp -rf frameworks opensource/
cp -rf filesystems opensource/
cp -rf mdm-init opensource/

mkdir -p opensource/android_compat/device/qcom
cp -rf android_compat/build opensource/android_compat/
cp -rf android_compat/device/qcom/msm8909 opensource/android_compat/device/qcom
cp -rf android_compat/device/qcom/common opensource/android_compat/device/qcom

mkdir -p opensource/hardware
cp -rf hardware/libhardware opensource/hardware

mkdir -p opensource/external
cp -rf external/bluetooth opensource/external
cp -rf external/bspatch opensource/external
cp -rf external/libselinux opensource/external
cp -rf external/libunwind opensource/external
cp -rf external/paycheck opensource/external
cp -rf external/safe-iop opensource/external

mkdir -p opensource/bootable/
cp -rf bootable/bootloader opensource/bootable/

mkdir -p opensource/mdm-ss-mgr/
cp -rf mdm-ss-mgr/reboot-daemon opensource/mdm-ss-mgr/

mkdir -p opensource/display/
cp -rf display/libdrm opensource/display/

tar -cvzf opensource.tgz opensource
git clean -xfd opensource/
