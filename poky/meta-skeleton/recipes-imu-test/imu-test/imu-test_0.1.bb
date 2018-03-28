SUMMARY = "Test IMU BMI160"
SECTION = "base"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${WORKDIR}/COPYRIGHT;md5=349c872e0066155e1818b786938876a4"

SRC_URI = "file://imu_bmi160_test \
	   file://imu_bmi160_test.c  \
	   file://iio_utils.h \
	   file://COPYRIGHT \
	   "

do_compile () {
	${CC} -I. ${WORKDIR}/imu_bmi160_test.c -o ${WORKDIR}/imu_bmi160_test
}


do_install () {
        install -d ${D}${sysconfdir}/init.d
        cat ${WORKDIR}/skeleton | \
          sed -e 's,/etc,${sysconfdir},g' \
              -e 's,/usr/sbin,${sbindir},g' \
              -e 's,/var,${localstatedir},g' \
              -e 's,/usr/bin,${bindir},g' \
              -e 's,/usr,${prefix},g' > ${D}${sysconfdir}/init.d/imu_bmi160_test
        chmod a+x ${D}${sysconfdir}/init.d/imu_bmi160_test

        install -d ${D}${sbindir}
        install -m 0755 ${WORKDIR}/imu_bmi160_test ${D}${sbindir}/
}

RDEPENDS_${PN} = "initscripts"

CONFFILES_${PN} += "${sysconfdir}/init.d/imu_bmi160_test"
