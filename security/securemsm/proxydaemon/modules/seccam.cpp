/**
 * Copyright (c) 2015-2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <errno.h>
#include <utils/Log.h>
#include "TAInterface.h"
#include "QSEEComAPI.h"
#include <linux/msm_ion.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>

#define NUM_ION_FDS (4)

static pthread_mutex_t seccam_qseecom_mtx = PTHREAD_MUTEX_INITIALIZER;

DECLARE_SEND_CMD_FUNC(seccam_send_cmd)
{
	int rv = 0;

	do {

		if (cmd == NULL) {
			ALOGE("cmd cannot be NULL");
			rv = -EINVAL;
			break;
		}

		if (cmdLen < sizeof(uint32_t)) {
			ALOGE("cmdLen too short: %u", cmdLen);
			rv = -EINVAL;
			break;
		}

		if (rspLen < sizeof(uint32_t)) {
			ALOGE("rspLen too short: %u", rspLen);
			rv = -EINVAL;
			break;
		}

		if (rsp == NULL) {
			ALOGE("rsp cannot be NULL");
			rv = -EINVAL;
			break;
		}

		pthread_mutex_lock(&seccam_qseecom_mtx);
		rv = QSEECom_send_cmd((struct QSEECom_handle*)handle, cmd, cmdLen, rsp, rspLen);
		pthread_mutex_unlock(&seccam_qseecom_mtx);

	} while (0);

	return rv;
}

static int32_t import_ION_handle(int32_t ion_dev_fd, int ion_buf_fd, struct ion_fd_data *ifd)
{
	int32_t ret = 0;

	do {
		if (!ifd) {
			ret = -EINVAL;
			break;
		}
		ifd->handle = 0;
		if (ion_dev_fd < 0) {
			ALOGE("Error ION device not opened.");
			ret = -EINVAL;
			break;
		}
		ALOGD("Mapping FD: %d", ion_buf_fd);
		ifd->fd = ion_buf_fd;
		ret = ioctl(ion_dev_fd, ION_IOC_IMPORT, ifd);
		if (ret) {
			ALOGE("Could not import ION buffer, fd=%d, err: %s (%d)", ion_buf_fd, strerror(ret), ret);
			break;
		}
		if (!ifd->handle) {
			ALOGE("Reference handle NULL.");
			ret = -EINVAL;
			break;
		}
		ALOGD("Handle: %x", ifd->handle);
		ifd->fd = -1;
		ret = ioctl(ion_dev_fd, ION_IOC_MAP, ifd);
		if (ret) {
			ALOGE("Could not map ION buffer, handle=%x, err: %s (%d)", ifd->handle, strerror(ret), ret);
			break;
		}
		if (ifd->fd == -1) {
			ALOGE("Invalid fd");
			ret = -EINVAL;
			break;
		}
	} while (0);

	if (ret) {
		if ((ifd) && (ifd->handle)) {
			struct ion_handle_data ihd = {ifd->handle};
			ioctl(ion_dev_fd, ION_IOC_FREE, &ihd);
			ifd->handle = 0;
			ifd->fd = -1;
		}
	}

	return ret;
}

static int32_t release_ION_handle(int32_t ion_dev_fd, struct ion_fd_data *ifd)
{
	int32_t ret = -EINVAL;
	struct ion_handle_data ihd = {0};

	if (!ifd) {
		return -EINVAL;
	}
	if (!ifd->handle) {
		ALOGE("Zero ION reference handle.");
		return -EINVAL;
	}
	if (ion_dev_fd < 0) {
		ALOGE("Error ION device not opened.");
		return -EINVAL;
	}
	ihd.handle = ifd->handle;
	if (ifd->fd >= 0) {
		ALOGD("Closing fd %d", ifd->fd);
		close(ifd->fd);
		ifd->fd = -1;
	}
	ALOGD("Releasing handle: %x", ihd.handle);
	ret = ioctl(ion_dev_fd, ION_IOC_FREE, &ihd);
	if (ret) {
		ALOGE("ION Memory FREE ioctl failed with ret = %d %s", ret, strerror(ret));
	}
	ifd->handle = 0;
	return ret;
}

DECLARE_SEND_MODIFIED_CMD_FUNC(seccam_send_modified_cmd)
{
	int32_t ion_dev_fd = -1;
	struct qseecom_app_info app_info;
	int32_t ret = 0;
	int32_t req_len = cmdLen;
	int32_t rsp_len = rspLen;
	uint32_t i;
	void* msgrsp;
	struct QSEECom_ion_fd_info l_info = {0};
	struct ion_fd_data ifd[NUM_ION_FDS] = {0};

	ALOGD("SendMdfCmd: start");

	if (!cmd || cmdLen < sizeof(uint32_t) || !rsp || rspLen < sizeof(uint32_t)) {
		ALOGE("SendMdfCmd: incorrect cmd parameters");
		return -1;
	}

	if (!info) {
		ALOGE("SendMdfCmd: incorrect ION buffer");
		return -1;
	}
	do{
		l_info = *info;

		ion_dev_fd = open("/dev/ion", O_RDONLY);
		if (ion_dev_fd < 0) {
			ALOGE("Error opening ION device: %s (%d)", strerror(errno), errno);
			ret = errno;
			break;
		}

		// import the ION FDs
		for (i = 0; i < NUM_ION_FDS; i++) {
			ALOGD("index %d, fd %d, offset %u", i, l_info.data[i].fd, l_info.data[i].cmd_buf_offset);
			if (l_info.data[i].fd > 0) {
				ret = import_ION_handle(ion_dev_fd, l_info.data[i].fd, &ifd[i]);
				if (ret) {
					ALOGE("Error importing ION fd: %d, %s(%d)", l_info.data[i].fd, strerror(ret), ret);
					break;
				}
				l_info.data[i].fd = ifd[i].fd;
				ALOGD("mapped index %d, fd %d, offset %u", i, l_info.data[i].fd, l_info.data[i].cmd_buf_offset);
			}
		}
		if (ret) break;


		if (req_len & QSEECOM_ALIGN_MASK) {
			req_len = QSEECOM_ALIGN(req_len);
		}

		if (rsp_len & QSEECOM_ALIGN_MASK) {
			rsp_len = QSEECOM_ALIGN(rsp_len);
		}

		msgrsp = (void*)(((struct QSEECom_handle*)handle)->ion_sbuffer + req_len);

		ALOGD("SendMdfCmd: req len = %d bytes",req_len);
		ALOGD("SendMdfCmd: rsp len = %d bytes",rsp_len);

		pthread_mutex_lock(&seccam_qseecom_mtx);
		memcpy(((struct QSEECom_handle*)handle)->ion_sbuffer, cmd, cmdLen);
		ret = QSEECom_send_modified_cmd_64((struct QSEECom_handle*)handle,
				((struct QSEECom_handle*)handle)->ion_sbuffer, req_len,
				msgrsp, rsp_len,
				&l_info);
		pthread_mutex_unlock(&seccam_qseecom_mtx);

	}while(0);

	// release the ION FDs
	for (i = 0; i < NUM_ION_FDS; i++) {
		if (ifd[i].handle) {
			release_ION_handle(ion_dev_fd, &ifd[i]); // ignore errors
			ifd[i].fd = -1;
			ifd[i].handle = 0;
		}
	}

	if (ion_dev_fd >= 0) {
		close(ion_dev_fd);
	}

	if (ret) {
		ALOGE( "SendMdfCmd: failed with ret = %d", ret);
	}
	else {
		if (rsp && rspLen) {
			memcpy(rsp, msgrsp, rspLen);
		}
		ALOGD("SendMdfCmd: PASS");
	}

	return ret;
}

