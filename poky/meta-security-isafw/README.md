# Automating Security Testing of OS & Firmware Images

This project is intended to automate various security tests that we perform for detection and remediation of vulnerabilities in our OS and firmware. This project also helps security team to understand:

* Traceability and components of firmware contents
* Customizing/automating security tooling process
* Better adhere to security best practices
* Prepare materials that are useful to yield more productive and less costly security assessments

Examples of some issues we want to detect early, as part of our build and development process in an automated way are:

* Hardcoded keys, secrets, tokens
* Open permissions on sensitive files/binaries
* Kernel security and binary hardening. Identifies security features binaries are compiled with
* CVE tracking of BOM and other installed components etc.
* Build time scanning of configuration files, applications and filesystem images
* Analyzing SetUID / SetGID bits

[Image Security Analysis Framework](https://github.com/intel/isafw) (ISAFW) is an open source security analysis framework. The purpose of ISAFW is to provide an extensible framework for analyzing different security aspects of OS images during the build process. The design intention of this framework is to stay build system independent and have a minimal interface towards it. The actual functional logic of the framework can be found in its plugins that can be created independently from each other.

Currently supported objects:

* **ISA\_package**. Represents a source package/recipe data for each package being built by the build system.
* **ISA\_pkg\_list**. Represents a list of binary packages information, such as package names and versions, that are being included into the OS image.
* **ISA\_kernel**. Represents information about the kernel to be included in the OS image, such as kernel configuration location.
* **ISA\_filesystem**. Represents information about the filesystem to be included in the OS image, such as its location and type.

The framework supports a number of callbacks that are invoked by the build system during different stages of package and OS image build. These callbacks are then forwarded for processing to the available ISAFW plugins that have registered for these callbacks. Plugins can do their own analysis on each stage of the build process and produce security reports.

Currently supported callbacks:

* ***process\_package(self, ISA\_package)***. Called per each source package that is being assembled by a build system
* ***process\_pkg\_list(self, ISA\_pkg\_list)***. Called once per each image assembled by a build system
* ***process\_kernel(self, ISA\_kernel)***. Called once per each image assembled by the build system.
* ***process\_filesystem(self, ISA\_filesystem)***. Called once per each file system that is being included into the image.

By default ISAFW supports the following plugins:

* **ISA\_cve\_plugin**. Plugin for checking CVE information for packages. Works on top of [cve-check-tool](https://github.com/clearlinux/cve-check-tool).
* **ISA\_la\_plugin**. Plugin for verifying licensing information for packages.
* **ISA\_cf\_plugin**. Plugin for analysing binary compilation flags on rootfs. Works on top of [checksec.sh](http://www.trapkit.de/tools/checksec.html) script.
* **ISA_kca\_plugin**. Plugin for analysing security aspects of kernel configuration (through analyzing kernel config file).
* **ISA\_fsa\_plugin**. A basic plugin for analysing image of file system (through analyzing file permissions).

Addition to the default ones we also integrated Anki [firmware analyzer tool](https://github.com/anki/security-afat) (AFAT) into the framework as a plugin. AFAT is a script for searching the extracted or mounted firmware file system for items of interest such as password, SSL configuration, SSL private keys, web servers, etc.

To enable ISAFW in the build system:

1. Check out the “merge-isafw” branch from vicos-oelinux git repository.
2. Uncomment ```INHERIT += "isafw"``` in the local.conf configuration file.
Build the OS, After compilation, ISAFW will generate a report folder in path -- ```poky/build/tmp-glibc/log/isafw-report_[TIMESTAMP]```

```
# checkout the isafw branch
git clone --recursive git@github.com:anki/vicos-oelinux.git
 
cd vicos-oelinux
 
git checkout daniel/merge-isafw
 
# edit local.conf file
vi poky/build/conf/local.conf
 
# remove the "#" infront of "INHERIT += "isafw" save and compile the vicos
source build/conf/set_bb_env.sh
build-victor-robot-image
```

#### ISAFW Report

* **cve\_report**: one CSV format file, one XML format file, and one HTML file lists known CVEs for each package and patch status
* **cfa\_problem report**: one text file and one XML file that lists security-related compiler flags for each package in the rootfs, such as RELRO,  stack canary, PIE, MPX, etc.
* **fsa\_problem report**: one text file and one XML file that lists writable files and SETUID bit set.
* **kca\_problem report**: one text file and one XML file that lists security-related kernel configuration flags (based on a whitelist that is customizable).
* **la\_problem report**: one text file and one XML file that lists all package license information (violation is reported based on a blacklist that is customizable).
* **afa\_full\_report**: a text file contains the output of AFAT.

#### ISAFW Internal

ISAFW is a simple Python library. The library is located in path ```meta-security-isafw/lib
/isafw```. The library interacts with Yocto through ```isafw.bbclass``` in the ```meta-security-isafw``` layer. By inheriting the ```isafw.bbclass``` in the build configuration, for each recipe Yocto adds a ```do_analysesource``` task before ```do_build```. In the process of build, the ```do_analysesource``` task initializes ISAFW, pass package variables, and invoke each plugin through callback functions.

The plugins for ISAW is located in path, ```meta-security-isafw/lib/isafw/isaplugins```. To extend, simply create a Python class and implement supported callback functions.

The ```meta-security-isafw``` depends on ```meta-python``` (ISAFW depends on ```python-lxml```, etc.). To avoid adding an entire ```meta-python``` layer into our code base we probably should keep ISAFW in a separate branch, since most of the builds do not need to enable it.



