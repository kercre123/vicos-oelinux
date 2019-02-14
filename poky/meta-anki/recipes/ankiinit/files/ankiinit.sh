#!/bin/sh
### This file should only contain hardware initalization commands that will only fail if the hardware is
##  bad. Anything that we can limp along without should be put somewhere else so that this script exits
##  with status 0 and the system init process can continue.

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

# Move RT FIFO tasks off core 0 to reduce timer interrupt induced jitter.
pgrep tty | xargs -I PID taskset -p 0xe PID

# TODO Move this power rail controll into the camera driver
CAM_REG_GPIO=83
if [ ! -d /sys/class/gpio/gpio$CAM_REG_GPIO ]; then
    echo $CAM_REG_GPIO > /sys/class/gpio/export
fi
echo out > /sys/class/gpio/gpio$CAM_REG_GPIO/direction
echo 1 > /sys/class/gpio/gpio$CAM_REG_GPIO/value
# End camera force hack

# Set spi buf size to size of LCD frame (184*96*2)
chmod 666 /sys/module/spidev/parameters/bufsiz
echo 35328 > /sys/module/spidev/parameters/bufsiz
chmod 444 /sys/module/spidev/parameters/bufsiz

# emr-cat prints a placeholder value even on error
SERIALNO=`/bin/emr-cat e`
HAVE_EMR=$?

# And put the serial number in properties
for i in `seq 1 5`;
do
    setprop ro.serialno $SERIALNO
    if test ! -z `getprop ro.serialno`; then break; fi
    sleep 1;
done

if [ $HAVE_EMR -eq 0 ]; then
  echo $SERIALNO > /sys/class/android_usb/android0/iSerial
fi

if [ -x /usr/bin/vic-christen ]; then
    /usr/bin/vic-christen
fi

/bin/hostname `getprop anki.robot.name | tr ' ' '-'`

# If we are coming up from a maintenance reboot, let the anki apps know
if [ -e /data/maintenance_reboot ]; then
    mv /data/maintenance_reboot /run/after_maintenance_reboot
fi

#Enable Ramoops
modprobe ramoops

exit 0
