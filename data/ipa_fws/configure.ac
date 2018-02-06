AC_PREREQ(2.61)
AC_INIT(ipa-fws, 1.0.0)
AM_INIT_AUTOMAKE([foreign])

AC_ARG_ENABLE([target-mdm9650],
	AC_HELP_STRING([--enable-target-mdm9650],
		[Enable conditional compile for target mdm9650]),
	[target_mdm9650="${enableval}"],
	target_mdm9650=no)

AC_ARG_ENABLE([target-sdx20],
	AC_HELP_STRING([--enable-target-sdx20],
		[Enable conditional compile for target sdx20]),
	[target_sdx20="${enableval}"],
	target_sdx20=no)

AM_CONDITIONAL([IS_MDM9650], [test "x$target_mdm9650" = xyes])
AM_CONDITIONAL([IS_SDX20], [test "x$target_sdx20" = xyes])

AC_OUTPUT(Makefile)
AC_OUTPUT
