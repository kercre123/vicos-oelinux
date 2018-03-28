SUMMARY = "Test IMU BMI160"
SECTION = "base"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${WORKDIR}/COPYRIGHT;md5=349c872e0066155e1818b786938876a4"


SRC_URI = "file://imu_bmi160_test.c \
	   file://iio_utils.c \
	   file://iio_utils.h \
	   file://iio_event_monitor.c \
	   file://lsiio.c \
	   file://COPYRIGHT \
	   "

do_compile () {
	${CC} -I. ${WORKDIR}/imu_bmi160_test.c  ${WORKDIR}/iio_utils.c -o ${WORKDIR}/imu_bmi160_test
	${CC} -I. ${WORKDIR}/lsiio.c  ${WORKDIR}/iio_utils.c -o ${WORKDIR}/lsiio
	${CC} -I. ${WORKDIR}/iio_event_monitor.c  ${WORKDIR}/iio_utils.c -o ${WORKDIR}/iio_event_monitor
}


do_install () {
        install -d ${D}${sbindir}
        install -m 0755 ${WORKDIR}/imu_bmi160_test ${D}${sbindir}/
        install -m 0755 ${WORKDIR}/lsiio ${D}${sbindir}/
        install -m 0755 ${WORKDIR}/iio_event_monitor ${D}${sbindir}/
}


