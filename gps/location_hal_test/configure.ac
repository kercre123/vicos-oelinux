# configure.ac -- Autoconf script for gps loc_api
#
# Process this file with autoconf to produce a configure script

# Requires autoconf tool later than 2.61
AC_PREREQ(2.61)
# Initialize the gps app  package version 1.0.0
AC_INIT([location-hal-tests],1.0.0)
# Does not strictly follow GNU Coding standards
AM_INIT_AUTOMAKE([foreign])
# Disables auto rebuilding of configure, Makefile.ins
AM_MAINTAINER_MODE
# Verifies the --srcdir is correct by checking for the path
AC_CONFIG_SRCDIR([location_hal_test.cpp])
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
PKG_CHECK_MODULES([LOCHAL], [loc-hal])
AC_SUBST([LOCHAL_CFLAGS])
AC_SUBST([LOCHAL_LIBS])

PKG_CHECK_MODULES([LOCPLA], [loc-pla])
AC_SUBST([LOCPLA_CFLAGS])
AC_SUBST([LOCPLA_LIBS])

PKG_CHECK_MODULES([LOCFLP], [location-flp])
AC_SUBST([LOCFLP_CFLAGS])
AC_SUBST([LOCFLP_LIBS])

PKG_CHECK_MODULES([LOCGEOFENCE], [location-geofence])
AC_SUBST([LOCGEOFENCE_CFLAGS])
AC_SUBST([LOCGEOFENCE_LIBS])

PKG_CHECK_MODULES([IZATCORE], [izat-core])
AC_SUBST([IZATCORE_CFLAGS])
AC_SUBST([IZATCORE_LIBS])

PKG_CHECK_MODULES([GPSUTILS], [gps-utils])
AC_SUBST([GPSUTILS_CFLAGS])
AC_SUBST([GPSUTILS_LIBS])

AC_ARG_WITH([libhardware_includes],
      AC_HELP_STRING([--with-libhardware-includes=@<:@dir@:>@],
         [Specify the location of the libhardware headers]),
      [libhardware_incdir=$withval],
      with_libhardware_includes=no)

if test "x$with_libhardware_includes" != "xno"; then
   CPPFLAGS="${CPPFLAGS} -I${libhardware_incdir}"
fi

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


AM_CONDITIONAL(USE_GLIB, test "x${with_glib}" = "xyes")

AC_CONFIG_FILES([ \
        Makefile \
        location-hal-tests.pc \
        ])

AC_OUTPUT
