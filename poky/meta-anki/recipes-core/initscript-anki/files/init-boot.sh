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
mount -t debugfs nodev /sys/kernel/debug

mkdir -p /var
mkdir -p /var/run

CMDLINE=$(cat /proc/cmdline)

ROOTFS="${CMDLINE##*root=}"
ROOTFS="${ROOTFS%% *}"

[ -z "${ROOTFS}" ] && fatal "ERROR: Kernel boot arg root=... not found"

# Enable power rails required for GPIO, LCD, IMU and Camera
echo 1 > /sys/kernel/debug/regulator/8916_l8/enable
echo 1 > /sys/kernel/debug/regulator/8916_l17/enable
echo 1 > /sys/kernel/debug/regulator/8916_l4/enable

if [ -z "${CMDLINE##*dm=*}" ]; then
	DM="${CMDLINE##*dm=\"}"
	DM="${DM%\" *}"
	DM_TABLE="${DM##*,}"

	# Power on hardware test and led states
	rampost

	echo "Setting up DM Verity device: $DM"
	dmsetup create system -r --table "$DM_TABLE" || fatal "ERROR: dmsetup failed"
else
	set -e
	# Confirm the serial number is on the white list or die
	SERIAL=`cat /sys/devices/soc0/serial_number`
	if test -z $SERIAL; then
		echo "Unable to get serial number"
		exit 1;
	fi
	if grep -qw $SERIAL unlock.list; then
		# Run user confirmation check for non verity devices
		rampost orange
	else
		rampost x
		exit 1;
	fi
fi

ROOT_MOUNT_POINT=/rootfs
mkdir -p "$ROOT_MOUNT_POINT"
mount "$ROOTFS" "$ROOT_MOUNT_POINT"
mount -n --move /proc "$ROOT_MOUNT_POINT/proc"
mount -n --move /sys "$ROOT_MOUNT_POINT/sys"
mount -n --move /dev "$ROOT_MOUNT_POINT/dev"

cd $ROOT_MOUNT_POINT

exec switch_root -c /dev/console $ROOT_MOUNT_POINT /sbin/init $CMDLINE || fatal "Couldn't switch_root"
