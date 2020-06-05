# vicos-oelinux

This repository contains the embedded OS that runs on victor hardware.
If you are lookingg for victor embedded firmware, robotics, animation and engine layers checkout [vicos](https://github.com/anki/victor).

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
are disabled for security. *All user builds are also perf builds.*

**User builds expect to use production signing keys**
* User build LK requires the production kernel signing key to be used to sign the kernel
* User build root file system requires OTA files to be signed with the production OTA signing key

Because user builds are basically not debuggable and require production keys, there are non-user versions of the user
builds.

| User                | Debuggable         |
|---------------------|--------------------|
| robot-user-image    | robot-perf-image   |
| robot-factory-image | robot-facdev-image |

Note that the robot-facdev-image build target as a non-user builds expects development keys, not production keys.

### Factory
Factory builds are intended as the out of box / recovery firmware programmed into the F slots of robots.
The primary difference for factory builds is that `/data` is always mounted as a tmpfs in factory builds so it is
erased every power cycles meaning the robot does not remember wifi configuration etc.


-------------------------------------------------------------------------------

## Making Unlock OTA files

To unlock production robots for development builds, a special OTA file
including an ABOOT unlocked to that QSN must be generated. Doing so
requires a few steps but can be largely automated. If this is done
wrong it has potential to BRICK a vector unit since we're messing with
code normally set by the factory and never modified again. Take
care to follow all instructions precisely

### LInux Configuration

You will need a linux box to do the build. This can be done in a VM
but will take a day to build unless you can assign something like 16
Gigs of memory and 8 cores to the VM an 100+ Gigs of disk space. If
you can do that see the VirtualBox branch of the
[vic-os-vm-config](https://github.com/anki/vic-os-vm-config)
repository for instructions on creating and priming a VM.

### SSH key forwarding

The anki git repositories are whitelisted to particular ssh keys within
your git profile. Rather than set up a new one, use ssh key forwarding
when sshing in to your VM:

    ssh-add # load your default key in to the keyring
    ssh -A grant@192.168.56.20 # open with forwarded connectin.

### Getting the source

Both OELinux and our victor repo aren't the best at sorting out stale
dependencies, particularly on a source rollback, so it's best to check
out a seperate version of the repository for making these files and
doing a clean build from scratch. Defer the submodule initialization
until after we've switched branches:

    cd src
    git clone git@github.com:anki/vicos-oelinux.git MP-UNLOCK
    cd MP-UNLOCK
    git checkout release/mp-unlock
    git submodule update --init --recursive
    git submodule update --recursive

### Initial Build - OTA only

Make sure we have a good image with the lasted production release that
works. If our buiild system isn't working correctly, we can still do a
factory reset.

1. Start VPN.
1. Build:
       cd poky # FROM MP-UNLOCK
        source build/conf/set_bb_env.sh
        build-victor-robot-factory-image # wait 40-50 minutes

1. Create OTA:
        cd $WORKSPACE/
         export OTA_KEY_PASS=<password> # Use a leading space to keep the password from being stored in HISTORY
        OTAKEY="ota_prod.key" ANKIDEV=0 UPDATE_VERSION=2.0.0 IMG_DIR=../poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf make -C ./ota/ all
        ls _build # look at what you just made

### OTA verification

If you want to make sure you built files suitable for deployment you
can do a test deployment before creating a file that can possibly
brick your device. You will not be able to run the code after
deployment because it is still signed with development keys, but you
can safely reset the robot when finished.

1. Copy the resulting file `vicos-2.0.0.ota` to your host machine
    where it's easier to connect to the robot.
1. Use a factory reset robot that has 0.9.0 Firmware. To check firmware version:
    * Put Vector a on powered docking station.
    * Wait for it to tell you to go to anki.com/v
    * Hit the backpack button twice to go in to BlueTooth pairing
    mode.
    * Raise the arm up and then down.
    * You should now see a screen with the version number 0.9.0.
    * If you don't see this hold down the backpack button for 20
      seconds and try again.
1. Deploy via mac-client or web client once that is restored. For
    mac-client:
    * Start a web server in the directory where you have the OTA:
      `python -m http.server` or for python2 `python -m
      SimpleHTTPServer`
    * Place robot in a powered dock and let it boot. Press the
      backpack button twice to go to pairing mode.
    * Connect with mac-client.
    * In mac-client, go to ap mode which seems to be more reliable
      `wifi-ap true`
    * Change your host machine wifi with SSID and PW listed in
      mac-client.
    * Get your IP from a terminal with `ifconfig | grep 192`
    * Go to http://<IP_ADDRESS>:8000 in your web browser, copy the
      link for the file.
    * In mac-client `ota-start <url>`
    * In mac-client `ota-progress` will hopefully show a status
      update.
    If you get an error consult the [Victor Error Codes](https://ankiinc.atlassian.net/wiki/spaces/ATT/pages/425492587/Victor+Error+Codes) document in
    confluence to debug.

If all goes well the device will reboot however it will hang at boot
with one white light on the back. This is fine. We're testing that the
code can be deployed to the client, but it can't run yet because the
unit isn't unlocked. Factory reset the robot before the next step.

### Full unlock build

#### Preqrequisite: APQ Sectools

You will need a copy of the special repository
apq8009-le-1-0-2_ap_standard_oem with a good version of
Qualcomm's sectools scripts.

**TODO:** *Security implications of taring up the sectools dir
instead of giving people the whole repo?*

#### Getting the QSN

Now that we have the system images we can unlock a robot. This
requires a different OTA file for each device since we need the
Qualcomm Serial Number to generate a valid file.

To get the QSN:

1. Connect to robot with ./mac-client
2. Run `logs` and wait for the log dump to finish.


#### Actual Build

1. Initialize environment:
        cd poky # FROM MP-UNLOCK
        source build/conf/set_bb_env.sh
1. Start a robot-perf-image build but cancel when it starts the task queue. This is just to change the build flavor
    parameters for the next step.
1. Make the unlock file: `cd ota` and `python3 mk_unlock.py -q<qsn>
   -sm --sectools ~/src/apq8009-le-1-0-2_ap_standard_oem/common/tools/sectools/` or
   `python3 mk_unlock.py -l <file with one QSN per line> -sm --sectools ~/src/apq8009-le-1-0-2_ap_standard_oem/common/tools/sectools/`

Note that signing the LK (`-s`) with cause the script to prompt for the signing password and making the OTA file (`-m`)
will cause it to prompt for the OTA signing key password.

When the build is complete you'll have a file in
`MP-UNLOCK/_build/unlock` with the robot's QSN in the filename. This
can be deployed as listed above with mac-client and after completion
you should have a unloced robot.
