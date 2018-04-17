#!/bin/sh
set -e

SLOT_SUFFIX="$(cat /proc/cmdline)"
SLOT_SUFFIX="${SLOT_SUFFIX##*androidboot.slot_suffix=}"
SLOT_SUFFIX="${SLOT_SUFFIX%% *}"

case "$SLOT_SUFFIX" in
  '_a')
    THIS_SLOT='a'
    ;;
  '_b')
    THIS_SLOT='b'
    ;;
  *)
    THIS_SLOT='f'
esac

bootctl $THIS_SLOT mark_successful
setprop ro.boot.successful 1

dmesg > /data/boot.log
