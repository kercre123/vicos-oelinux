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
LIC_FILES_CHKSUM = "file://start-stop-daemon.c;endline=26;md5=d12ebbc165f2c86fec2dd1bf92653167"

SRC_URI = "git://github.com/daleobrien/start-stop-daemon.git;protocol=https"

# Modify these as desired
PV = "1.0+git${SRCPV}"
SRCREV = "${AUTOREV}"

S = "${WORKDIR}/git"

FILES_${PN} += "start-stop-daemon"

PROVIDES = "start-stop-daemon"

# NOTE: no Makefile found, unable to determine what needs to be done

do_configure () {
	# Specify any needed configure commands here
	:
}

do_compile () {
	echo "###########################################"
	echo "## compiling start-stop-daemon            "
	echo "###########################################"
	${CC} ${CFLAGS} ${LDFLAGS} start-stop-daemon.c -o start-stop-daemon
}

do_install () {
	echo "###########################################"
	echo "## installing start-stop-daemon            "
	echo "###########################################"
	echo -n "install -d ${D}${base_sbindir}..."
	install -d ${D}${base_sbindir} && echo "OK" || echo "ERROR"
	echo -n "install -m 0755 ${S}/start-stop-daemon ${D}${base_binsdir}..."
	install -m 0755 ${S}/start-stop-daemon ${D}${base_sbindir} && echo "OK" || echo "ERROR"
}

