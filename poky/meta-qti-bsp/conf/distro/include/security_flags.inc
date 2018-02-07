# Pull in the default security flags
require ${COREBASE}/meta/conf/distro/include/security_flags.inc

# Modify the primary security flag definitions
SECURITY_CFLAGS = "-fstack-protector-strong -pie -fpie -Wa,--noexecstack"
SECURITY_PIC_CFLAGS = "-fstack-protector-strong -fPIC -Wa,--noexecstack"
SECURITY_LDFLAGS = "-Wl,-z,relro,-z,now,-z,noexecstack"

# GPS OSS packages to use -fPIC in security flags instead of -fpie
SECURITY_CFLAGS_pn-loc-hal = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-pla = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-stub = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-gps-utils = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-net-iface = "${SECURITY_PIC_CFLAGS}"
