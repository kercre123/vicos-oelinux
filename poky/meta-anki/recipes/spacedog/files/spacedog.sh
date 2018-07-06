#!/bin/sh

# This script has been tested with the busybox versions of df, tail, tr and cut

# mount point of target device
MOUNT_POINT="/data"

# check disk space every $INTERVAL seconds
INTERVAL="1"

# capcity limit in %
LIMIT="90"

require()
{
    for a in "$@"; do
        if [ ! -x "$(command -v "$a")" ]; then
            echo "ERROR: executable '$a' is required in PATH, please install!" && exit 1
        fi
    done
}

_export_df()
{
    export DEVICE=$1    && shift
    export BLOCKS=$1    && shift
    export USED=$1      && shift
    export AVAILABLE=$1 && shift
    export CAPACITY=$1  && shift
    export CAPACITY=${CAPACITY%%\%}  && shift
}

acquire()
{
# SELF TEST!
#    local size=829660
#    local cap=$(( RANDOM % 100 ))
#    local used=$(( cap * size / 100 ))
#    local avail=$(( size - used ))
#    echo /dev/mmcblk0p32 $size $used $avail ${cap}% /data
#    _export_df /dev/mmcblk0p32 $size $used $avail $cap /data

    _export_df $(df -P $MOUNT_POINT|tail -n 1 |tr -s ' ' ' ')
}

require df tail tr cut

acquire

while true; do
    acquire
    echo "CAPACITY: $CAPACITY"

    if [ "$CAPACITY" -ge "$LIMIT" ]; then
        echo "We're at $LIMIT% capacity or above!! Actually at $CAPACITY%! TAKE ACTION!"
    fi

    sleep $INTERVAL
done
