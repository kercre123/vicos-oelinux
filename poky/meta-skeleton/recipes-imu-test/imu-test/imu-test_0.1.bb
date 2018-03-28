SUMMARY = "Test IMU BMI160"
SECTION = "base"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${WORKDIR}/COPYRIGHT;md5=349c872e0066155e1818b786938876a4"


SRC_URI = "file://imu_bmi160_test.c \
	   file://iio_utils.c \
	   file://iio_utils.h \
	   file://COPYRIGHT \
	   "

do_compile () {
	${CC} -I. ${WORKDIR}/imu_bmi160_test.c  ${WORKDIR}/iio_utils.c -o ${WORKDIR}/imu_bmi160_test
}


do_install () {
        install -d ${D}${sbindir}
        install -m 0755 ${WORKDIR}/imu_bmi160_test ${D}${sbindir}/
}


