C_PREREQ(2.61)
AC_INIT([init_mss], 1.0.0)
AM_INIT_AUTOMAKE([-Wall gnu foreign])
AM_MAINTAINER_MODE
AC_CONFIG_HEADER([config.h])
AC_CONFIG_MACRO_DIR([m4])

AC_PROG_CC
AM_PROG_CC_C_O
AM_PROG_AS
AC_PROG_LIBTOOL
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET

PKG_PROG_PKG_CONFIG

AC_ARG_ENABLE(indefinite-sleep,
      AC_HELP_STRING([--enable-indefinite-sleep],
         [Allow init_mss to sleep forever]),
         [indefinite_sleep=${enableval}])

AM_CONDITIONAL(SLEEP_INDEFINITE, [test "x${indefinite_sleep}" = "xyes"])

AC_SUBST([CFLAGS])
AC_SUBST([CC])
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
