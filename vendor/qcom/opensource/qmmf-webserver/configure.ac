AC_PREREQ(2.61)
AC_INIT([qmmf-support], 1.0.0)
AM_INIT_AUTOMAKE([foreign])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Check for programs
LT_INIT
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

AS_CASE([$host],
        [arm*], [ARM=yes],
        [ARM=no]
)

ARM=yes

[CPPFLAGS="$CPPFLAGS -std=c++14 -Wall -Wextra -Werror"]

AC_SUBST([CFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([LIBS])

AC_ARG_WITH(sanitized-headers,
      AS_HELP_STRING([--with-sanitized-headers=DIR],
         [Specify the location of the sanitized Linux headers]),
      [CPPFLAGS="$CPPFLAGS -idirafter $withval"])

AC_ARG_WITH([camcommon],
      AC_HELP_STRING([with-camcommon=@<:@dir@:>@],
         [Specify the location of Camera stack common headers]),
      [camcommon_incdir=$withval])

AC_ARG_WITH([camifaceinc],
      AC_HELP_STRING([with-camifaceinc=@<:@dir@:>@],
         [Specify the location of Camera interface headers]),
      [camiface_incdir=$withval])

AC_ARG_WITH([exif],
      AC_HELP_STRING([with-exif=@<:@dir@:>@],
         [Specify the location of exif headers]),
      [exif_incdir=$withval])

AC_ARG_WITH([omxcore],
      AC_HELP_STRING([with-omxcore=@<:@dir@:>@],
         [Specify the location of OMX core headers]),
      [omxcore_incdir=$withval])

AC_ARG_WITH([openmax],
      AC_HELP_STRING([with-openmax=@<:@dir@:>@],
         [Specify the location of OpenMAX headers]),
      [openmax_incdir=$withval])

AC_ARG_WITH([sqlite],
      AC_HELP_STRING([with-sqlite=@<:@dir@:>@],
         [Specify the location of SQLite headers]),
      [sqlite_incdir=$withval])

AC_ARG_WITH([camerahal],
      AC_HELP_STRING([with-camerahal=@<:@dir@:>@],
         [Specify the location of CameraHal]),
      [camerahal_libdir=$withval],
      with_camerahal=no)

AC_SUBST([CAMERASTACKCOMMON_CFLAGS], "${camcommon_incdir}")
AC_SUBST([CAMERAINTERFACE_CFLAGS], "${camiface_incdir}")
AC_SUBST([EXIF_CFLAGS], "${exif_incdir}")
AC_SUBST([OMXCORE_CFLAGS], "${omxcore_incdir}")
AC_SUBST([OPENMAX_CFLAGS], "${openmax_incdir}")

AC_SUBST([SQLITE_CFLAGS], "${sqlite_incdir}")

if test "x$with_camerahal" != "xno"; then
   AC_SUBST([CAMERAHAL_CFLAGS], "${camerahal_libdir}")
fi

AC_CONFIG_FILES([ \
    Makefile \
    rtspinterface/Makefile \
    muxinterface/Makefile \
    jpegencoder/Makefile \
    database/Makefile \
    vaminterface/Makefile \
    httpinterface/Makefile \
])
AC_OUTPUT
