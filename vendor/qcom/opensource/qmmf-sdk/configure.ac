AC_PREREQ(2.61)
AC_INIT([qmmf-recorder], 1.0.0)
AM_INIT_AUTOMAKE([foreign subdir-objects -Wall -Wextra -Werror])
AC_CONFIG_SRCDIR([Makefile.am])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

m4_ifdef([AM_PROG_AR], [AM_PROG_AR])

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

AC_ARG_WITH(sanitized-headers,
      AS_HELP_STRING([--with-sanitized-headers=DIR],
         [Specify the location of the sanitized Linux headers]),
      [CPPFLAGS="$CPPFLAGS -idirafter $withval"])

AC_ARG_WITH([gralloc_library],
      AC_HELP_STRING([--with-gralloc-library=@<:@dir@:>@],
         [Specify the location of the Gralloc library]),
      [gralloc_libdir=$withval],
      with_gralloc_library=no)

if test "x$with_gralloc_library" != "xno"; then
   AC_SUBST([LIBGRALLOC], "${gralloc_libdir}")
fi

AC_ARG_WITH([mm_core],
      AC_HELP_STRING([with-mm-core=@<:@dir@:>@],
         [Specify the location of the mm-core library]),
      [mm_core_libdir=$withval],
      with_mm_core_library=no)

if test "x$with_mm_core_library" != "xno"; then
   AC_SUBST([MMCORE_CFLAGS], "${mm_core_libdir}")
fi

AC_ARG_WITH([drone-target],
      AC_HELP_STRING([with-drone-target=@<:@dir@:>@],
         [Specify drone target]),
      [drone_target=$withval],
      drone_target=false)

if test "x$drone_target" == "xtrue" ; then
[CPPFLAGS="$CPPFLAGS -D_DRONE_"]
fi

AC_ARG_WITH([camerahal],
      AC_HELP_STRING([with-camerahal=@<:@dir@:>@],
         [Specify the location of CameraHal]),
      [camerahal_libdir=$withval],
      with_camerahal=no)

if test "x$with_camerahal" != "xno"; then
   AC_SUBST([CAMERAHAL_CFLAGS], "${camerahal_libdir}")
fi

[CPPFLAGS="$CPPFLAGS -std=c++14 -Wall -Wextra -Werror"]

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

# TODO functions have unused input parameters
[CPPFLAGS="$CPPFLAGS -Wno-unused-parameter"]
# TODO this warning suppress issue in lib gralloc private
[CPPFLAGS="$CPPFLAGS -Wno-multichar"]
# TODO remove this when gcc version is updated
[CPPFLAGS="$CPPFLAGS -Wno-missing-field-initializers"]
# TODO define ANDROID macro to resolve redefinition of LOGE by MMDebugMsg.h
[CPPFLAGS="$CPPFLAGS -DANDROID"]

AC_ARG_WITH([basemachine],
        AC_HELP_STRING([with-basemachine=@<:@machine name@:>@],
        [Specify the Machine name]),
        [case "${withval}" in
                apq8053) multicam_enable=true ;;
                *) multicam_enable=false ;;
        esac],
        [multicam_enable=false])
AM_CONDITIONAL([ENABLE_360], [test x$multicam_enable = xtrue])

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

AC_SUBST([CAMERASTACKCOMMON_CFLAGS], "${camcommon_incdir}")
AC_SUBST([CAMERAINTERFACE_CFLAGS], "${camiface_incdir}")
AC_SUBST([EXIF_CFLAGS], "${exif_incdir}")
AC_SUBST([OMXCORE_CFLAGS], "${omxcore_incdir}")
AC_SUBST([OPENMAX_CFLAGS], "${openmax_incdir}")

AC_SUBST([CFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([LIBS])

AC_SUBST([CAMERASTACKCOMMON_CFLAGS], "${camcommon_incdir}")
AC_SUBST([CAMERAINTERFACE_CFLAGS], "${camiface_incdir}")
AC_SUBST([EXIF_CFLAGS], "${exif_incdir}")
AC_SUBST([OMXCORE_CFLAGS], "${omxcore_incdir}")
AC_SUBST([OPENMAX_CFLAGS], "${openmax_incdir}")

AC_CONFIG_FILES([ \
    Makefile \
    common/overlay/Makefile \
    common/cameraadaptor/Makefile \
    common/audio/src/client/Makefile \
    common/audio/src/service/Makefile \
    common/audio/test/samples/Makefile \
    common/codecadaptor/Makefile \
    common/avqueue/Makefile \
    plugins/Makefile \
    avcodec/Makefile \
    display/src/client/Makefile \
    display/src/service/Makefile \
    recorder/src/client/Makefile \
    recorder/src/service/post-process/modules/test/Makefile \
    recorder/src/service/post-process/modules/camera-hal-jpeg/Makefile \
    recorder/src/service/post-process/modules/camera-hal-reproc/Makefile \
    recorder/src/service/post-process/modules/jpeg-encoder/Makefile \
    recorder/src/service/post-process/modules/algo/Makefile \
    recorder/src/service/Makefile \
    player/src/client/Makefile \
    player/src/service/Makefile \
    system/src/client/Makefile \
    system/src/service/Makefile \
    system/test/samples/Makefile \
    qmmf-server/Makefile \
    qmmf-sdk.pc \
    recorder/test/samples/Makefile \
    recorder/test/gtest/Makefile \
    recorder/test/timelapse/Makefile \
    recorder/test/videotimelapse/Makefile \
    display/test/samples/Makefile \
    display/test/gtest/Makefile \
    player/test/demuxer/Makefile \
    player/test/samples/Makefile \
    player/test/gtest/Makefile \
    player/test/transcode_app/Makefile \
])
AC_OUTPUT
