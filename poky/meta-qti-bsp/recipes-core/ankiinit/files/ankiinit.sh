#!/bin/sh
### This file should only contain hardware initalization commands that will only fail if the hardware is
##  bad. Anything that we can limp along without should be put somewhere else so that this script exits
##  with status 0 and the system init process can continue.

set -e


# Enable power rails required for GPIO, LCD, IMU and Camera but not yet controlled by device tree
# TODO Configure device tree to properly control these regulators.
echo 1 > /sys/kernel/debug/regulator/8916_l8/enable
echo 1 > /sys/kernel/debug/regulator/8916_l17/enable
echo 1 > /sys/kernel/debug/regulator/8916_l4/enable

# @nathan-anki  - to stop overheating, limit CPU to 533MHz, RAM to 400MHz
echo 533333 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
echo disabled > /sys/kernel/debug/msm_otg/bus_voting  # This prevents USB from pinning RAM to 400MHz
echo 0 > /sys/kernel/debug/msm-bus-dbg/shell-client/update_request
echo 1 > /sys/kernel/debug/msm-bus-dbg/shell-client/mas
echo 512 > /sys/kernel/debug/msm-bus-dbg/shell-client/slv
echo 0 > /sys/kernel/debug/msm-bus-dbg/shell-client/ab
echo active clk2 0 1 max 400000 > /sys/kernel/debug/rpm_send_msg/message # Max RAM freq in KHz = 400MHz
echo 1 > /sys/kernel/debug/msm-bus-dbg/shell-client/update_request
#End clock speed limits

# TODO Move this power rail controll into the camera driver
CAM_REG_GPIO=83
if [ ! -d /sys/class/gpio/gpio$CAM_REG_GPIO ]; then
    echo $CAM_REG_GPIO > /sys/class/gpio/export
fi
echo out > /sys/class/gpio/gpio$CAM_REG_GPIO/direction
echo 1 > /sys/class/gpio/gpio$CAM_REG_GPIO/value
# End camera force hack

# Print the ID on the face
SERIALNO=`/bin/emr-cat e`

# And put the serial number in properties
for i in `seq 1 5`;
do
    setprop ro.serialno $SERIALNO
    if test ! -z `getprop ro.serialno`; then break; fi
    sleep 1;
done

if [ -x /usr/bin/vic-christen ]; then
    /usr/bin/vic-christen
fi

echo $SERIALNO > /sys/class/android_usb/android0/iSerial
echo 2 1 g $SERIALNO | /system/bin/display
echo 2 2 b `getprop persist.anki.robot.name` | /system/bin/display

exit 0
