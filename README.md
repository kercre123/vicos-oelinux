# vicos-oelinux

THIS BRANCH OF THE CODE IS FOR MAKING NORMAL FACTORY AND DEV/OSKR
UNLOCK IMAGES. YOU ONLY WANT THIS TO BUILD A NEW BASE MANUFACTUING
IMAGE OR UNLOCK PROD ROBOTS FOR EITHER DEV/OSKR MODE.

THIS ASSUMES YOU HAVE BUILT A **NORMAL** OTA UPDATE BEFORE. IF YOU HAVEN'T
CHECK OUT THE MASTER BRANCH AND FOLLOW THE INSTRUCTIONS SO YOU UNDERSTAND
HOW THE SYSTEM WORKS.

## Practical matters.

This is a delicate build with potential to brick robots if done incorrectly.
The best thing to do is get the code build, tested, and locked down, then
just run the part listed as **Actual Build** on a day to day basis to generate
the new images.

If you do need to bring up a new machine, make sure to test
extensively before rolling out an image.

## Building the Factory image
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

### Fixing the victor project dependencies

Becuase this branch is out-of-date compared to the current branch the
project dependencies installation is broken and complicated to fix. It
is easier to preconfigure the `victor` repository so that the dependencies
are already in place.

```
cd anki/victor
./install_old_externals.sh
```

See the victor README for more details.

### Initial Build And Prep

Make sure we have a good image with the lasted production release that
works. If our buiild system isn't working correctly, we can still do a
factory reset.

1. Start VPN.
1. Build:
       cd poky # FROM MP-UNLOCK
        source build/conf/set_bb_env.sh
        build-victor-robot-factory-image # wait 40-50 minutes

## OPTION 1: Making a factory image

This is relatively easy as we are simply trying to build the three
partitions needed for factory recovery that will be installed via
QDL. We do not need to go through all the hack steps to build an OTA
to edit the recovery partition.

Sign the boot image with the production key:

```
cd ota
make prodsign
# provide appropriate credentials
```

Build the ABOOT image. Writeup currently incomplete.

```
cd poky
source build/conf/set_bb_env.bb
# set flavor
# run bitbake command for lk.
```

## OPTION 2: Making Unlock OTA files

To unlock production robots for development builds, a special OTA file
including an ABOOT unlocked to that QSN must be generated. Doing so
requires a few steps but can be largely automated. If this is done
wrong it has potential to BRICK a vector unit since we're messing with
code normally set by the factory and never modified again. Take
care to follow all instructions precisely

### Set OSKR Unlock flavor if wanted

By default this will be a Dev bot build pointing to our dev stack and
with a warning screen. If we want an OSKR build we need to change the
signature on the system image:

```
cd ota
make oskrsign
```

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
1. To set the proper environment variables, you need to run a build task and then quit it after it fires up.

    For a DEV image: `build-robot-perf-image`

    For an OSKR image: `build-robot-perfoskr-image`

1. Make the unlock file: `cd ota` and `python3 mk_unlock.py -q<qsn>
   -sm --sectools ~/src/apq8009-le-1-0-2_ap_standard_oem/common/tools/sectools/` or
   `python3 mk_unlock.py -l <file with one QSN per line> -sm --sectools ~/src/apq8009-le-1-0-2_ap_standard_oem/common/tools/sectools/`

Note that signing the LK (`-s`) with cause the script to prompt for the signing password and making the OTA file (`-m`)
will cause it to prompt for the OTA signing key password.

When the build is complete you'll have a file in
`MP-UNLOCK/_build/unlock` with the robot's QSN in the filename. This
can be deployed as listed above with mac-client and after completion
you should have a unloced robot.

### Verification - After first full build.

1. Verify the signatures match. A make task will check both boot and ABOOT match.
    ```
    cd ota
    make verify-boot-dev # for DEV builds
    make verify-boot-oskr # for OSKR builds
    ```

2. Verify you properly checked out the correct submodules.

    ```
    cd anki/victor
    git log # check logs to make sure we're not on mainline, maybe a tag
    ```

### Testing new image on existing DEV/OSKR bot.

The security checks in the update script normally prevent you from
re-updating a DEV or OSKR unlocked bot. However these are just softare
checks and can be disabled manually in the robots
`/anki/dev/update-engine`

```diff
diff --git a/platform/update-engine/update-engine.py b/platform/update-engine/update-engine.py
index 9c8fdcd357..c37833bb3a 100755
--- a/platform/update-engine/update-engine.py
+++ b/platform/update-engine/update-engine.py
@@ -567,8 +567,8 @@ def validate_new_os_version(current_os_version, new_os_version, cmdline):
     new_os_version_suffix = m.groups()[0]
     m = os_version_regex.match(current_os_version)
     current_os_version_suffix = m.groups()[0]
-    if new_os_version_suffix != current_os_version_suffix:
-        die(216, "Update from " + current_os_version + " to " + new_os_version + " not allowed")
+#    if new_os_version_suffix != current_os_version_suffix:
+#        die(216, "Update from " + current_os_version + " to " + new_os_version + " not allowed")
     if LooseVersion(new_os_version) < LooseVersion(current_os_version):
         die(216, "Downgrade from " + current_os_version + " to " + new_os_version + " not allowed")
     return
@@ -620,11 +620,11 @@ def update_from_url(url):
         validate_new_os_version(current_os_version, next_boot_os_version, cmdline)
         if DEBUG:
             print("Updating to version {}".format(next_boot_os_version))
-        if is_dev_robot(cmdline):
-            if not manifest.getint("META", "ankidev"):
-                die(214, "Ankidev OS can't install non-ankidev OTA file")
-        elif manifest.getint("META", "ankidev"):
-            die(214, "Non-ankidev OS can't install ankidev OTA file")
+#        if is_dev_robot(cmdline):
+#            if not manifest.getint("META", "ankidev"):
+#                die(214, "Ankidev OS can't install non-ankidev OTA file")
+#        elif manifest.getint("META", "ankidev"):
+#            die(214, "Non-ankidev OS can't install ankidev OTA file")
         # Mark target unbootable
         if not call(['/bin/bootctl', current_slot, 'set_unbootable', target_slot]):
             die(202, "Could not mark target slot unbootable")
```

And then kick off the update manually with this command. You will also need to reboot manually when its complete.

`UPDATE_IMAGE_DEBUG=True UPDATE_IMAGE_URL=http://localhost:8000/mpunlock/vicos-5.0.0-312675720.ota ./update-engine`