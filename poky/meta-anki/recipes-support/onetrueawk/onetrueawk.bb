# Recipe created by recipetool
# This is the basis of a recipe and may need further editing in order to be fully functional.
# (Feel free to remove these comments when editing.)
#
# Unable to find any files that looked like license statements. Check the accompanying
# documentation and source headers and set LICENSE and LIC_FILES_CHKSUM accordingly.
#
# NOTE: LICENSE is being set to "CLOSED" to allow you to at least start building - if
# this is not accurate with respect to the licensing of the software being built (it
# will not be in most cases) you must specify the correct value before using this
# recipe for anything other than initial testing/development!
LICENSE = "PD"
LIC_FILES_CHKSUM = "file://LICENSE;md5=2297f1fc5d91aa55090c2fe11c064a84"

# Our source is based on 
SRC_URI = "git://github.com/onetrueawk/awk.git;protocol=https \
           file://0001-use-BUILD_CC-to-build-intermediate-maketab-binary.patch \
           "
PV = "git${SRCPV}"
SRCREV = "2af1c7c65841101bda4f0e4651d6c5b77d7fd462"
S = "${WORKDIR}/git"

FILES_${PN} += "awk"
RPROVIDES_${PN} = "awk"
PROVIDES = "awk"

inherit update-alternatives

ALTERNATIVE_${PN} = "awk"
ALTERNATIVE_TARGET[awk] = "${base_bindir}/awk.${PN}"
ALTERNATIVE_PRIORITY = "101"

do_configure () {
}

do_compile () {
	make CC="${CC}" CFLAGS="${CFLAGS}" LDFLAGS="${LDFLAGS}" YACC="yacc -d "
}

do_install () {
	install -d ${D}${base_bindir}
	install -m 0755 ${S}/a.out ${D}${base_bindir}/awk.${PN}
}

