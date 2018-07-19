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

Because user builds are basically not debugable, there are non-user versions of the user builds.

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

To unlock production robots for development builds, a special OTA file including an ABOOT unlocked to that QSN must be
generated. Doing so requires a few steps but can be largely automated:

1. Build a robot-perf-image flavor build.
2. Download a copy of the MP firmware apq8009-robot-boot.img and apq8009-robot-sysfs.img into `_build/`
3. `cd ota`
4. Resign the boot image with the dev key: `make resign-dev`
5. `python3 mk_unlock.py -q<qsn> -sm` or `python3 mk_unlock.py -l <file with one QSN per line> -sm`

Note that signing the LK (`-s`) with cause the script to prompt for the signing password and making the OTA file (`-m`)
will cause it to prompt for the OTA signing key password.
