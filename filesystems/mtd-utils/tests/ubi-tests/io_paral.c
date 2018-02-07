/*
 * Copyright (c) International Business Machines Corp., 2006
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Author: Artem B. Bityutskiy
 *
 * This test does a lot of I/O to volumes in parallel.
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "libubi.h"
#define PROGRAM_NAME "io_paral"
#include "common.h"
#include "helpers.h"

#define ITERATIONS  (1024 * 1)

static libubi_t libubi;
static struct ubi_dev_info dev_info;
static const char *node;
static int vol_size;

int vol_lebs = 0;
int threads_num = 0;
static struct ubi_mkvol_request *reqests;
static char **vol_name;
static char **vol_nodes;
static unsigned char **wbufs;
static unsigned char **rbufs;

static int update_volume(int vol_id, int bytes)
{
	int i, fd, ret, written = 0, rd = 0;
	char *vol_node = vol_nodes[vol_id];
	unsigned char *wbuf = wbufs[vol_id];
	unsigned char *rbuf = rbufs[vol_id];

	fd = open(vol_node, O_RDWR);
	if (fd == -1) {
		failed("open");
		errorm("cannot open \"%s\"\n", vol_node);
		return -1;
	}

	for (i = 0; i < bytes; i++)
		wbuf[i] = rand() % 255;
	memset(rbuf, '\0', bytes);

	ret = ubi_update_start(libubi, fd, bytes);
	if (ret) {
		failed("ubi_update_start");
		errorm("volume id is %d", vol_id);
		goto err_close;
	}

	while (written < bytes) {
		int to_write = rand() % (bytes - written);

		if (to_write == 0)
			to_write = 1;

		ret = write(fd, wbuf + written, to_write);
		if (ret != to_write) {
			failed("write");
			errorm("failed to write %d bytes at offset %d "
			       "of volume %d", to_write, written,
			       vol_id);
			errorm("update: %d bytes", bytes);
			goto err_close;
		}

		written += to_write;
	}

	close(fd);

	fd = open(vol_node, O_RDONLY);
	if (fd == -1) {
		failed("open");
		errorm("cannot open \"%s\"\n", node);
		return -1;
	}

	/* read data back and check */
	while (rd < bytes) {
		int to_read = rand() % (bytes - rd);

		if (to_read == 0)
			to_read = 1;

		ret = read(fd, rbuf + rd, to_read);
		if (ret != to_read) {
			failed("read");
			errorm("failed to read %d bytes at offset %d "
			       "of volume %d", to_read, rd, vol_id);
			goto err_close;
		}

		rd += to_read;
	}

	if (memcmp(wbuf, rbuf, bytes)) {
		errorm("written and read data are different");
		goto err_close;
	}

	close(fd);
	return 0;

err_close:
	close(fd);
	return -1;
}

static void *update_thread(void *ptr)
{
	int vol_id = (long)ptr, i;

	for (i = 0; i < ITERATIONS; i++) {
		int ret, bytes = (rand() % (vol_size - 1)) + 1;
		int remove = !(rand() % 16);

		/* From time to time remove the volume */
		if (remove) {
			ret = ubi_rmvol(libubi, node, vol_id);
			if (ret) {
				failed("ubi_rmvol");
				errorm("cannot remove volume %d", vol_id);
				return NULL;
			}
			ret = ubi_mkvol(libubi, node, &reqests[vol_id]);
			if (ret) {
				failed("ubi_mkvol");
				errorm("cannot create volume %d", vol_id);
				return NULL;
			}
		}

		ret = update_volume(vol_id, bytes);
		if (ret)
			return NULL;
	}

	return NULL;
}

static void *write_thread(void *ptr)
{
	int ret, fd, vol_id = (long)ptr, i;
	char *vol_node = vol_nodes[vol_id];
	unsigned char *wbuf = wbufs[vol_id];
	unsigned char *rbuf = rbufs[vol_id];

	fd = open(vol_node, O_RDWR);
	if (fd == -1) {
		failed("open");
		errorm("cannot open \"%s\"\n", vol_node);
		return NULL;
	}

	ret = ubi_set_property(fd, UBI_VOL_PROP_DIRECT_WRITE, 1);
	if (ret) {
		failed("ubi_set_property");
		errorm("cannot set property for \"%s\"\n", vol_node);
	}

	for (i = 0; i < ITERATIONS * vol_lebs; i++) {
		int j, leb = rand() % vol_lebs;
		off_t offs = dev_info.leb_size * leb;

		ret = ubi_leb_unmap(fd, leb);
		if (ret) {
			failed("ubi_leb_unmap");
			errorm("cannot unmap LEB %d", leb);
			break;
		}

		for (j = 0; j < dev_info.leb_size; j++)
			wbuf[j] = rand() % 255;
		memset(rbuf, '\0', dev_info.leb_size);

		ret = pwrite(fd, wbuf, dev_info.leb_size, offs);
		if (ret != dev_info.leb_size) {
			failed("pwrite");
			errorm("cannot write %d bytes to offs %lld, wrote %d",
				dev_info.leb_size, offs, ret);
			break;
		}

		/* read data back and check */
		ret = pread(fd, rbuf, dev_info.leb_size, offs);
		if (ret != dev_info.leb_size) {
			failed("read");
			errorm("failed to read %d bytes at offset %d "
			       "of volume %d", dev_info.leb_size, offs,
			       vol_id);
			break;
		}

		if (memcmp(wbuf, rbuf, dev_info.leb_size)) {
			errorm("written and read data are different");
			break;
		}
	}

	close(fd);
	return NULL;
}

static void *safe_malloc(size_t size)
{
	void *ptr = malloc(size);
	if (!ptr){
		errorm("Memory allocation failed");
		exit(1);
	}
	memset(ptr, 0, size);
	return ptr;
}

int main(int argc, char * const argv[])
{
	int i, ret;
	pthread_t *threads;

	seed_random_generator();
	if (initial_check(argc, argv))
		return 1;

	if (argc == 4){
		node = argv[1];
		threads_num = atoll(argv[2]);
		vol_lebs = atoll(argv[3]);
	} else {
		errorm("Usage: ./io_paral <ubi_dev> <# threads> <# lebs per thread>");
		exit(1);
	}

	threads = safe_malloc(threads_num * sizeof(pthread_t));
	reqests = safe_malloc((threads_num + 1) *
				sizeof(struct ubi_mkvol_request));
	wbufs = safe_malloc((threads_num + 1) * sizeof(char *));
	rbufs = safe_malloc((threads_num + 1) * sizeof(char *));

	vol_name = safe_malloc((threads_num + 1) * sizeof(char *));
	for(i = 0; i < threads_num + 1; ++i)
		vol_name[i] = safe_malloc(100 * sizeof(char));

	vol_nodes = safe_malloc((threads_num + 1) * sizeof(char *));
	for(i = 0; i < threads_num + 1; ++i)
		vol_nodes[i] = safe_malloc((sizeof(UBI_VOLUME_PATTERN) + 99) *
					    sizeof(char));

	libubi = libubi_open();
	if (libubi == NULL) {
		failed("libubi_open");
		return 1;
	}

	if (ubi_get_dev_info(libubi, node, &dev_info)) {
		failed("ubi_get_dev_info");
		goto close;
	}

	/*
	 * Create 1 volume more than threads count. The last volume
	 * will not change to let WL move more stuff.
	 */

	vol_size = dev_info.leb_size * vol_lebs;
	for (i = 0; i <= threads_num; i++) {
		reqests[i].alignment = 1;
		reqests[i].bytes = vol_size;
		reqests[i].vol_id = i;
		sprintf(vol_name[i], PROGRAM_NAME":%d", i);
		reqests[i].name = vol_name[i];
		reqests[i].vol_type = UBI_DYNAMIC_VOLUME;
		if (i == threads_num)
			reqests[i].vol_type = UBI_STATIC_VOLUME;
		sprintf(vol_nodes[i], UBI_VOLUME_PATTERN, dev_info.dev_num, i);

		if (ubi_mkvol(libubi, node, &reqests[i])) {
			failed("ubi_mkvol");
			goto remove;
		}

		wbufs[i] = malloc(vol_size);
		rbufs[i] = malloc(vol_size);
		if (!wbufs[i] || !rbufs[i]) {
			failed("malloc");
			goto remove;
		}

		ret = update_volume(i, vol_size);
		if (ret)
			goto remove;
	}

	for (i = 0; i < threads_num / 2; i++) {
		ret = pthread_create(&threads[i], NULL, &write_thread,
				     (void *)(long)i);
		if (ret) {
			failed("pthread_create");
			goto remove;
		}
	}
	for (i = threads_num / 2; i < threads_num; i++) {
		ret = pthread_create(&threads[i], NULL, &update_thread,
					(void *)(long)i);
		if (ret) {
			failed("pthread_create");
			goto remove;
		}
	}

	for (i = 0; i < threads_num; i++)
		pthread_join(threads[i], NULL);

	for (i = 0; i <= threads_num; i++) {
		if (ubi_rmvol(libubi, node, i)) {
			failed("ubi_rmvol");
			goto remove;
		}
		if (wbufs[i])
			free(wbufs[i]);
		if (rbufs[i])
			free(rbufs[i]);
		wbufs[i] = NULL;
		rbufs[i] = NULL;
	}

	libubi_close(libubi);
	return 0;

remove:
	for (i = 0; i <= threads_num; i++) {
		ubi_rmvol(libubi, node, i);
		if (wbufs[i])
			free(wbufs[i]);
		if (rbufs[i])
			free(rbufs[i]);
		if (vol_name[i])
			free(vol_name[i]);
		if (vol_nodes[i])
			free(vol_nodes[i]);
		wbufs[i] = NULL;
		rbufs[i] = NULL;
	}

	if (threads)
		free(threads);
	if (reqests)
		free(reqests);
	if (vol_name)
		free(vol_name);
	if (vol_nodes)
		free(vol_nodes);
	if (wbufs)
		free(wbufs);
	if (rbufs)
		free(rbufs);

close:
	libubi_close(libubi);
	return 1;
}
