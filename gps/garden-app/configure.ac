# configure.ac -- Autoconf script for gps garden-app
#
# Process this file with autoconf to produce a configure script

# Requires autoconf tool later than 2.61
AC_PREREQ(2.61)
# Initialize the gps garden-app package version 1.0.0
AC_INIT([garden-app],1.0.0)
# Does not strictly follow GNU Coding standards
AM_INIT_AUTOMAKE([foreign])
# Disables auto rebuilding of configure, Makefile.ins
AM_MAINTAINER_MODE
# Verifies the --srcdir is correct by checking for the path
AC_CONFIG_SRCDIR([GARDEn/test_android_gps.cpp])
# defines some macros variable to be included by source
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

# Checks for libraries.
PKG_CHECK_MODULES([QMI], [qmi])
AC_SUBST([QMI_CFLAGS])
AC_SUBST([QMI_LIBS])

PKG_CHECK_MODULES([QMIF], [qmi-framework])
AC_SUBST([QMIF_CFLAGS])
AC_SUBST([QMIF_LIBS])

PKG_CHECK_MODULES([LOCPLA], [loc-pla])
AC_SUBST([LOCPLA_CFLAGS])
AC_SUBST([LOCPLA_LIBS])
PKG_CHECK_MODULES([LOC], [loc-hal])
AC_SUBST([LOC_CFLAGS])
AC_SUBST([LOC_LIBS])
PKG_CHECK_MODULES([GPSUTILS], [gps-utils])
AC_SUBST([GPSUTILS_CFLAGS])
AC_SUBST([GPSUTILS_LIBS])
PKG_CHECK_MODULES([GTI], [garden-test-interfaces])
AC_SUBST([GTI_CFLAGS])
PKG_CHECK_MODULES([LOCVZW], [loc-vzw])
AC_SUBST([LOCVZW_CFLAGS])
AC_SUBST([LOCVZW_LIBS])
PKG_CHECK_MODULES([ULP2], [ulp2])
AC_SUBST([ULP2_CFLAGS])
AC_SUBST([ULP2_LIBS])
PKG_CHECK_MODULES([LOCFLP], [location-flp])
AC_SUBST([LOCFLP_CFLAGS])
AC_SUBST([LOCFLP_LIBS])

PKG_CHECK_MODULES([LOCGLUE], [loc-glue])
AC_SUBST([LOCGLUE_CFLAGS])
AC_SUBST([LOCGLUE_LIBS])

PKG_CHECK_MODULES([IZATCORE], [izat-core])
AC_SUBST([IZATCORE_CFLAGS])
AC_SUBST([IZATCORE_LIBS])

#PKG_CHECK_MODULES([LOCHALTESTSHIM], [loc-hal-test-shim])
#AC_SUBST([LOCHALTESTSHIM_CFLAGS])
#AC_SUBST([LOCHALTESTSHIM_LIBS])
PKG_CHECK_MODULES([LOCATIONSERVICE], [location-service])
AC_SUBST([LOCATIONSERVICE_CFLAGS])
AC_SUBST([LOCATIONSERVICE_LIBS])

PKG_CHECK_MODULES([EVENTOBSERVER], [event-observer])
AC_SUBST([EVENTOBSERVER_CFLAGS])
AC_SUBST([EVENTOBSERVER_LIBS])

AC_ARG_WITH([core_includes],
      AC_HELP_STRING([--with-core-includes=@<:@dir@:>@],
         [Specify the location of the core headers]),
      [core_incdir=$withval],
      with_core_includes=no)

if test "x$with_core_includes" != "xno"; then
   CPPFLAGS="${CPPFLAGS} -I${core_incdir}"
fi

AC_SUBST([CPPFLAGS])

AC_ARG_WITH([glib],
      AC_HELP_STRING([--with-glib],
         [enable glib, building HLOS systems which use glib]))

if (test "x${with_glib}" = "xyes"); then
        AC_DEFINE(ENABLE_USEGLIB, 1, [Define if HLOS systems uses glib])
        PKG_CHECK_MODULES(GTHREAD, gthread-2.0 >= 2.16, dummy=yes,
                                AC_MSG_ERROR(GThread >= 2.16 is required))
        PKG_CHECK_MODULES(GLIB, glib-2.0 >= 2.16, dummy=yes,
                                AC_MSG_ERROR(GLib >= 2.16 is required))
        GLIB_CFLAGS="$GLIB_CFLAGS $GTHREAD_CFLAGS"
        GLIB_LIBS="$GLIB_LIBS $GTHREAD_LIBS"

        AC_SUBST(GLIB_CFLAGS)
        AC_SUBST(GLIB_LIBS)
fi

#AC_ARG_WITH([qmi-framework],
#      AC_HELP_STRING([--with-qmi-framework],
#         [enable qmi-framework, building HLOS systems which use qmi-framework]))
#
#if (test "x${with_qmi-framework}" = "xyes"); then
#        AC_DEFINE(ENABLE_USEQMIF, 1, [Define if HLOS systems uses qmi-framework])
#        PKG_CHECK_MODULES([QMIF], [qmi-framework])
#        QMIF_CFLAGS="$QMIF_CFLAGS"
#        QMIF_LIBS="$QMIF_LIBS"
#        AC_SUBST([QMIF_CFLAGS])
#        AC_SUBST([QMIF_LIBS])
#
#fi

#AC_ARG_WITH([mcm-core],
#      AC_HELP_STRING([--with-mcm-core],
#         [enable mcm-core, building HLOS systems which use mcm-core]))
#
#if (test "x${with_mcm-core}" = "xyes"); then
#        AC_DEFINE(ENABLE_USEMCMCORE, 1, [Define if HLOS systems uses mcm-core])
#        PKG_CHECK_MODULES([MCMCORE], [mcm-core])
#        MCMCORE_CFLAGS="$MCMCORE_CFLAGS"
#        MCMCORE_LIBS="$MCMCORE_LIBS"
#        AC_SUBST([MCMCORE_CFLAGS])
#        AC_SUBST([MCMCORE_LIBS])
#fi

#AC_ARG_ENABLE(target,
#        [AS_HELP_STRING([--enable-target=TARGET], [Specify the target product to build])],
#        [TARGET=$enableval],
#        [TARGET=none]
#)

#if test "x$TARGET" = "x9615-cdp"; then
#        TARGET_IS_9615=yes
#        TARGET_OS_LE=yes
#        AC_MSG_NOTICE([Target is 9615.])
#        PKG_CHECK_MODULES([MCMCORE], [mcm-core])
#        AC_SUBST([MCMCORE_CFLAGS])
#        AC_SUBST([MCMCORE_LIBS])
#        PKG_CHECK_MODULES([LOCMCMTESTSHIM], [loc-mcm-test-shim])
#        AC_SUBST([LOCMCMTESTSHIM_CFLAGS])
#        AC_SUBST([LOCMCMTESTSHIM_LIBS])
#        PKG_CHECK_MODULES([LOCMCMQMITESTSHIM], [loc-mcm-qmi-test-shim])
#        AC_SUBST([LOCMCMQMITESTSHIM_CFLAGS])
#        AC_SUBST([LOCMCMQMITESTSHIM_LIBS])
#
#fi

#elif test "x$TARGET" = "xmdm9625"; then
#        TARGET_IS_9625=yes
#        TARGET_OS_LE=yes
#        AC_MSG_NOTICE([Target is 9625.])
#elif test "x$TARGET" = "xmdm9635"; then
#        TARGET_IS_9635=yes
#        TARGET_OS_LE=yes
#        AC_MSG_NOTICE([Target is 9635.])
#elif test "x$TARGET" = "xmsm8610"; then
#        TARGET_IS_8610=yes
#        AC_MSG_NOTICE([Target is 8610.])
#elif test "x$TARGET" = "xmsm8960"; then
#        TARGET_IS_8960=yes
#        AC_MSG_NOTICE([Target is 8960.])
#elif test "x$TARGET" = "xmsm8974"; then
#        TARGET_IS_8974=yes
#        AC_MSG_NOTICE([Target is 8974.])
#fi

#AM_CONDITIONAL([TARGET_OS_LE], [test "x$TARGET" = "x9615-cdp"])
#AM_CONDITIONAL(TARGET_OS_LE, test "x${TARGET_IS_9625}" = "xyes")
#AM_CONDITIONAL(TARGET_OS_LE, test "x${TARGET_IS_9635}" = "xyes")

#AM_CONDITIONAL(TARGET_OS_LT, test "x${TARGET_IS_8960}" = "xyes")
#AM_CONDITIONAL(TARGET_OS_LT, test "x${TARGET_IS_8974}" = "xyes")
#AM_CONDITIONAL(TARGET_OS_LT, test "x${TARGET_IS_8610}" = "xyes")

AM_CONDITIONAL(USE_GLIB, test "x${with_glib}" = "xyes")

AC_CONFIG_FILES([ \
        Makefile \
        GARDEn/Makefile \
        garden-app.pc \
        ])

AC_OUTPUT
