#! /bin/sh

destdir=/mnt/sdcard/

umount_partition()
{
        if grep -qs "^/dev/$1 " /proc/mounts ; then
                umount -lf "${destdir}";
        fi
}

mount_partition()
{
        if [ ! -d "${destdir}" ]; then
            mkdir "${destdir}"
        fi
        if ! mount -t auto "/dev/$1" "${destdir}"; then
                # failed to mount
                exit 1
        fi
}

check_if_boot_dev()
{
                cat /proc/cmdline | grep androidboot.bootdevice >/dev/null || return $?
                a=$(cat /proc/cmdline); a=${a#*androidboot.bootdevice=}; boot_dev=${a%% *}; 
                real_sysfs_path=`realpath /sys/class/block/$1`
                echo "echo $real_sysfs_path | grep /sys/devices/ | grep $boot_dev"
                echo $real_sysfs_path | grep /sys/devices/ | grep $boot_dev >/dev/null #>/dev/kmsg && echo "*** DEBUG ***: $1 is a boot device" >/dev/kmsg
                return $?
}


create_symlink()
{
                real_sysfs_path=`realpath /sys/class/block/$1`
                partition_name=`cat $real_sysfs_path/uevent | grep PARTNAME`
                partition_name="${partition_name##*PARTNAME=}"
                partition_name="${partition_name% }"
                mkdir -p /dev/block/bootdevice/by-name/
                partition_name=/dev/block/bootdevice/by-name/$partition_name
                target_dev=/dev/$1
                #echo "*** DEBUG *** ln -s $target_dev $partition_name"  >/dev/kmsg
                ln -s $target_dev $partition_name
}

//echo "ACTION=${ACITION} 1=$1" >/dev/kmsg

case "${ACTION}" in
add|"")
        ! check_if_boot_dev $1 && umount_partition ${1} && mount_partition ${1} && break
        create_symlink $1 &
        ;;
remove)
        umount_partition ${1}
        ;;
esac

