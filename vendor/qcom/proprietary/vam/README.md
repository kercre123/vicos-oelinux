# VAM

This project allows you to run a Video Analytics Simulator on the target device.

Folder structure & file description

    .
    ├── Makefile.am
          Top-level Makefile
    ├── configure.ac
          Top-level config file
    ├── vam_lib
          Builds libVAManager.so
    ├── vam_utils
    │   ├── JSON
              Builds libjson_apis.so
    │   ├── VAMReport
              A program to compare metadata & ground truth
    ├── vam_sim
          Video Analytics Simulator
    ├── test_engines
          Built-in engines to test basic features of VAM
    ├── engine_generator
          Template for custom engine
    └── README
          This file.

# VA Simulator on Target

## LE Setup

[Instructions for Linux Build Setup] (http://qwiki.qualcomm.com/quic/LE_Unification)

## Installation

Yocto Jethro is used in this project to build Linux images.

[Starting Guide and Manual] (http://www.yoctoproject.org/docs/2.0.1/mega-manual/mega-manual.html)

Assuming LE Setup and Yocto Jethro installation have been completed, the following steps will help you run VA Sim on the 8053 LE Target.

## Compile vam

To generate below files for the target,

* VASim executable
* VAMReport executable
* JSON .so files
* VAManager .so files
* test engine libs

do the following:

1. `$ cd <workspace>/poky`

2. Clone vam repository.

3. To compile:

   `$ cd build`

   `$ bitbake -c cleanall vam-test`

   `$ bitbake -c compile vam-test`

   Any errors can be troubleshooted using the following:

   `$ bitbake -v -f -c compile vam-test`

5. To push files to target, do `$ bitbake vam-test`


To run the VA Sim test, see vam_sim.





