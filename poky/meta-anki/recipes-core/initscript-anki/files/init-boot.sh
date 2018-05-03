#!/bin/sh

fatal() {
	echo "$@"
	exit 1
}

export PATH=/sbin:/bin:/usr/sbin:/usr/bin

mkdir -p /proc
mkdir -p /sys
mkdir -p /dev
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs none /dev

mkdir -p /var
mkdir -p /var/run

CMDLINE=$(cat /proc/cmdline)

ROOTFS="${CMDLINE##*root=}"
ROOTFS="${ROOTFS%% *}"

[ -z "${ROOTFS}" ] && fatal "ERROR: Kernel boot arg root=... not found"

if [ -z "${CMDLINE##*dm=*}" ]; then
	DM="${CMDLINE##*dm=\"}"
	DM="${DM%\" *}"
	DM_TABLE="${DM##*,}"

	echo "Setting up DM Verity device: $DM"
	dmsetup create system -r --table "$DM_TABLE" || fatal "ERROR: dmsetup failed"
else
	echo "######################################################################"
	echo "## DEVELOPMENT BUILD: LOOK TOWARDS THE CAMERA, SMILE, AND HIT ENTER ##"
	echo "######################################################################"
	read challenge
	if [ "$challenge" == "shell" ]; then
		exec sh
	fi
	echo -n "## running face recognition"
	sleep 1 && echo -n "." && sleep 1 && echo -n "." && sleep 1 && echo -n "." && sleep 1
	echo -n "passed!"
	echo
fi

ROOT_MOUNT_POINT=/rootfs
mkdir -p "$ROOT_MOUNT_POINT"
mount "$ROOTFS" "$ROOT_MOUNT_POINT"
mount -n --move /proc "$ROOT_MOUNT_POINT/proc"
mount -n --move /sys "$ROOT_MOUNT_POINT/sys"
mount -n --move /dev "$ROOT_MOUNT_POINT/dev"

cd $ROOT_MOUNT_POINT

exec switch_root -c /dev/console $ROOT_MOUNT_POINT /sbin/init $CMDLINE || fatal "Couldn't switch_root"
