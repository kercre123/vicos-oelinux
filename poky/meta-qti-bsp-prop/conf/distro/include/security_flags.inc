require ${COREBASE}/meta-qti-bsp/conf/distro/include/security_flags.inc

# GPS Prop packages to use -fPIC in security flags instead of -fpie
SECURITY_CFLAGS_pn-drplugin-client = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-drplugin-server = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-drplugin = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-event-observer = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-gdtap-adapter = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-izat-core = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-izat-client-api = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-lbs-core = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-base-util = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-externaldr = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-mq-client = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-vzw = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-location-flp = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-lowi-client  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-slim-client  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-slim-utils  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-ulp2  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-glue  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-loc-hal-test-shim  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-location-service  = "${SECURITY_PIC_CFLAGS}"
SECURITY_CFLAGS_pn-data-items  = "${SECURITY_PIC_CFLAGS}"

# Anki
SECURITY_CFLAGS_pn-ankibluetoothd = "${SECURITY_PIE_CFLAGS}"