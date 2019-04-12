#
# Copyright (C) 2008 OpenedHand Ltd.
#

SUMMARY = "Profiling tools"
LICENSE = "MIT"

PR = "r3"

PACKAGE_ARCH = "${MACHINE_ARCH}"

inherit packagegroup

# lttng-ust uses sched_getcpu() which is not there on uclibc
# for some of the architectures it can be patched to call the
# syscall directly but for x86_64 __NR_getcpu is a vsyscall
# which means we can not use syscall() to call it. So we ignore
# it for x86_64/uclibc

LTTNGUST = "lttng-ust"
LTTNGUST_libc-uclibc = ""
LTTNGUST_aarch64 = ""

LTTNGTOOLS = "lttng-tools"
LTTNGTOOLS_aarch64 = ""

LTTNGMODULES = "lttng-modules"
LTTNGMODULES_aarch64 = ""

BABELTRACE = "babeltrace"
BABELTRACE_aarch64 = ""

RDEPENDS_${PN} = "\
    ${LTTNGUST} \
    ${LTTNGTOOLS} \
    ${LTTNGMODULES} \
    ${BABELTRACE} \
    "
