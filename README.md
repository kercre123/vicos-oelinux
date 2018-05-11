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
