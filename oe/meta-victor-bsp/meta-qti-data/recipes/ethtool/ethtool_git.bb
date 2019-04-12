inherit autotools
SUMMARY = "ethtool is the standard Linux utility for managing Ethernet Drivers"
DESCRIPTION = "ethtool is the standard Linux utility for controlling network\
drivers and hardware, particularly for wired Ethernet devices"
HOMEPAGE = "https://www.kernel.org/pub/software/network/ethtool/"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;beginline=1;endline=340;md5=b234ee4d69f5fce4486a80fdaf4a4263"

PR = "r2"

SRCREV = "27798121da5093fcbd0537fbd4c9a1177c620f55"
#SRC_URI = "git://git.kernel.org/pub/scm/network/ethtool/ethtool.git;protocol=git"
SRC_URI = "${CAF_GIT}/platform/external/ethtool;protocol=git;branch=korg/master"

SRC_URI[md5sum] = "7e94dd958bcd639aad2e5a752e108b24"
SRC_URI[sha256sum] = "562e3cc675cf5b1ac655cd060f032943a2502d4d59e5f278f02aae92562ba261"

S = "${WORKDIR}/git"
