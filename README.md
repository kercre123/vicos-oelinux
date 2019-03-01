# vicos-oelinux

This repository contains the embedded OS that runs on victor hardware.
If you are looking for victor embedded firmware, robotics, animation and engine layers checkout [vicos](https://github.com/anki/victor).

## Build Instructions

The following steps require a Linux host setup to build vicos-oelinux, e.g. `vmosGamma.ankicore.com`.

Clone git repository:
```
git clone --recursive git@github.com:anki/vicos-oelinux.git
```

Start Build:
```
cd vicos-oelinux/poky
source build/conf/set_bb_env.sh
build-victor-robot-image
```

-------------------------------------------------------------------------------

## Build Flavors

We currently have 5 different build flavors for the Victor OS.

*build-victor-*

* robot-image
* robot-perf-image
* robot-user-image
* robot-factory-image
* robot-facdev-image

These permute three switches

### Perf
Perf builds are optimized more for performance than non-perf builds, particularly in the Linux Kernel. At the moment
*robot-image* is the only non-perf build flavor.

### User
User build are as intended for software released to actual users. Debugging features such as SSH, ADB and fastboot
are disabled for security. **All user builds are also perf builds.**

Because user builds are basically not debuggable, there are non-user versions of the user builds.

| User                | Debuggable         |
|---------------------|--------------------|
| robot-user-image    | robot-perf-image   |
| robot-factory-image | robot-facdev-image |

### Factory
Factory builds are intended as the out of box / recovery firmware programmed into the F slots of robots.
The primary difference for factory builds is that `/data` is always mounted as a tmpfs in factory builds so it is
erased every power cycles meaning the robot does not remember wifi configuration etc.


-------------------------------------------------------------------------------

## Making Unlock OTA files

Checkout the `release/mp-unlock` branch and read this file there.

-------------------------------------------------------------------------------


## SoC / WiFi / Modem firmware

The APQ8009 is a complex SoC with a number of processors besides the quad core ARM processor which runs our Linux
operating system. The other processors such as the modem DSP and wlan DSP require their own firmware and configuration
files.

### wlan / cnss_proc

The wlan DSP, or *cnss_proc* as Qualcomm calls it, has it's own firmware which comes from the APQ8009 repository,
<vmosAlpha.ankicore.com:/git/apq8009-le-1-0-2_ap_standard_oem.git> not the vicos-oelinux repository and it gets built
into the *modem* partition which is mounted under Linux at `/firmware`. We do not have source code for this firmware. It
comes as blobs from Qualcomm.

The wlan also requires configuration files which are in the repository under `android_compat/device/qcom/msm8909`
and get baked into the root file system image at `/lib/firmware/wlan/prima/`. These configuration files are loaded by
the wlan Linux driver and contain values which have been tuned for Victor to pass FCC and other testing. These
configuration files *must not be touched without discussion with the Anki hardware department*.

#### Updating firmware

TL;DR: We **don't**.

Because the firmware lives on the *modem* partition and not in the root file system, it is not touched by OTAs. If at
some point in the future we discover we do need to update the wlan firmware, we will need to figure out how to do this
via a post install step after OTA and work out it's implications for the F slot OS and do a lot of testing before we
consider releasing an update.

### modem_proc

The modem DSP loads firmware for the GPS receiver even though we have no GPS phy or antenna. We have received customized
GPS firmware from Qualcomm to work around a hardware bug in the PMIC which is triggered by our electrical design.
Refer to Qualcomm CreatePoint support `Case Number 03482936` for details. The customized firmware is located in the
APQ8009 repository under `modem_proc`. Since the GPS firmware isn't actually used, we shouldn't ever need to update it.
If we do need to for some reason, we need to make certain that any new firmware also includes the patches for the PMIC
issue.

## Updating `VICTOR_COMPAT_VERSION` for breaking changes

If you make a change that will break the `vic-*` apps under `anki/victor`, then you must increment the `VICTOR_COMPAT_VERSION`.  This will make it so that developers working on `victor.git`, do not try to
accidentally deploy to a robot running an old version.  Similarly, if they try to deploy code from an out of date branch onto a newer robot, they will get an error to rebase their branch and try again.
