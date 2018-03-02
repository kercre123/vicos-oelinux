#!/usr/bin/env sh

set -x

BOOT_URI=$1
SYS_URI=$2

SLOT_SUFFIX=`tr ' ' '\n' < /proc/cmdline | awk -F= /androidboot.slot_suffix/'{print $2}' | xargs`

BOOT_STEM=/dev/block/bootdevice/by-name/boot
SYS_STEM=/dev/block/bootdevice/by-name/system

case "$SLOT_SUFFIX" in
  '_a')
    THIS_SLOT='a'
    DST_SLOT='b'
    ;;
  '_b')
    THIS_SLOT='b'
    DST_SLOT='a'
    ;;
  *)
    THIS_SLOT='f'
    DST_SLOT='a'
esac

set -xe

BOOT_DST="${BOOT_STEM}_${DST_SLOT}"
SYS_DST="${SYS_STEM}_${DST_SLOT}"

bootctl $THIS_SLOT set_unbootable $DST_SLOT

curl $BOOT_URI | gunzip -c | dd of=$BOOT_DST
curl $SYS_URI  | gunzip -c | dd of=$SYS_DST

bootctl $THIS_SLOT set_active $DST_SLOT
