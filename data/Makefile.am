# Makefile.am - Automake script for data
#
ACLOCAL_AMFLAGS = -I m4

dirs :

dirs = netmgr/src
dirs += qdi/src
dirs += dsi_netctrl/src
dirs += tests/dsi_netctrl/src
dirs += qdp/src

if !IS_MDM9607_PSM
  dirs += radish
if !TARGET_IS_8X96
  dirs += mobileap/ipc
  dirs += mobileap/server/src mobileap/client/src qti/src
  dirs += mobileap/webserver/qcmap_web_client/src mobileap/webserver/cgi
  dirs += mobileap/eMBMS_multiclient/qmi_ip_multiclient/src
  dirs += mobileap/eMBMS_multiclient/tunneling/src
if !TARGET_IS_9607
  dirs += ipacm-diag/src
  dirs += ipacm-perf/src
  dirs += qti_socksv5/src
  dirs += port-bridge
endif
endif
endif

SUBDIRS = $(dirs)

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = data.pc
EXTRA_DIST = $(pkgconfig_DATA)
