#!/bin/sh
### BEGIN INIT INFO
# Provides:          resetrobot                                                                                                              $
# Required-Start:    $remote_fs $all
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:
## END INIT INFO


# The /data partition is mounted via a service and not
# immediately at boot via fstab. Wait until we see an
# expected directory so we know it's mounted
while [ -z "`getprop anki.robot.name`" ];
do
    logger -t resetrobot "Robot doesn't have a unique name yet. Sleeping for 5 seconds..."
    sleep 5
done

logger -t resetrobot "Wiping all data from robot"

logger -t resetrobot "Cleaning up EMR"

dd if=/dev/zero of=/dev/mmcblk0p29 seek=1 bs=16

logger -t resetrobot "wiping boot_a and boot_b"
dd if=/dev/zero of=/dev/mmcblk0p21
dd if=/dev/zero of=/dev/mmcblk0p24

logger -t resetrobot "Forcing clear user data flag"

touch /run/wipe-data

logger -t resetrobot "rebooting"
/sbin/reboot
