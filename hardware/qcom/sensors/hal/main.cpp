/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <signal.h>
#include <pthread.h>
#include <poll.h>

#include "BstSensor.h"
#include "bsx_android.h"

#include "sensors_poll_context.h"


static const struct sensor_t *sSensorList = NULL;
static int sensorsNum = 0;

static int sensors__get_sensors_list(struct sensors_module_t* module,
        struct sensor_t const** list)
{
    (void) module;
    *list = sSensorList;
    return sensorsNum;
}

#if defined(SENSORS_DEVICE_API_VERSION_1_4)
static int sensors__set_operation_mode(unsigned int mode)
{
    (void)mode;
    return 0;
}
#endif


/*****************************************************************************/

sensors_poll_context_t::sensors_poll_context_t()
{
    bst_sensor = BstSensor::getInstance();
    sensorsNum = bst_sensor->get_sensorlist(&sSensorList);
}

//for cppcheck "noCopyConstructor"
sensors_poll_context_t::sensors_poll_context_t(const sensors_poll_context_t & other)
{
    *this = other;
}

sensors_poll_context_t::~sensors_poll_context_t()
{
    BstSensor::destroy();
}

int sensors_poll_context_t::activate(int handle, int enabled)
{
    return bst_sensor->activate(handle, enabled);
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns)
{
    return bst_sensor->setDelay(handle, ns);
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int nbEvents = 0;
    int n = 0;
    int bst_evncnt = 0;
    struct pollfd extended_mPollFds[1];

    extended_mPollFds[0].fd = bst_sensor->HALpipe_fd[0];
    extended_mPollFds[0].events = POLLIN;
    extended_mPollFds[0].revents = 0;

    do {
        // see if we have some leftover from the last poll()
        if(extended_mPollFds[0].revents & POLLIN){
            bst_evncnt = bst_sensor->read_events(data, count);

            if (bst_evncnt < count) {
                // no more data for next time
                extended_mPollFds[0].revents = 0;
            }
            count -= bst_evncnt;
            nbEvents += bst_evncnt;
            data += bst_evncnt;
        }

        if (count) {
            // we still have some room, so try to see if we can get
            // some events immediately or just wait if we don't have
            // anything to return
            do {
                n = poll(extended_mPollFds, 1, nbEvents ? 0 : -1);
            } while (n < 0 && errno == EINTR);

            if (n < 0) {
                return -errno;
            }
        }
        // if we have events and space, go read them
    } while (n && count);

    return nbEvents;
}

int sensors_poll_context_t::batch(int handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{
    return bst_sensor->batch(handle, flags, sampling_period_ns, max_report_latency_ns);
}

int sensors_poll_context_t::flush(int handle)
{
    return bst_sensor->flush(handle);
}

#if defined(SENSORS_DEVICE_API_VERSION_1_4)
int sensors_poll_context_t::inject_sensor_data(const sensors_event_t *data)
{
    return bst_sensor->inject_sensor_data(data);
}
#endif

/*****************************************************************************/
/*fixed route, just copy from other sensor HAL code*/
static int poll__close(struct hw_device_t *dev)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    if (ctx)
    {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
        int handle, int enabled)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    return ctx->pollEvents(data, count);
}

static int poll__batch(struct sensors_poll_device_1 *dev,
        int handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    return ctx->batch(handle, flags, sampling_period_ns, max_report_latency_ns);
}

static int poll__flush(struct sensors_poll_device_1 *dev, int sensor_handle)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    return ctx->flush(sensor_handle);
}

#if defined(SENSORS_DEVICE_API_VERSION_1_4)
static int poll__inject_sensor_data(struct sensors_poll_device_1 *dev, const sensors_event_t *data)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *) dev;
    return ctx->inject_sensor_data(data);
}
#endif


/*****************************************************************************/
/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
        struct hw_device_t** device)
{
    int status = -EINVAL;
    sensors_poll_context_t *dev = new sensors_poll_context_t();
    (void) id;

    memset(&dev->device, 0, sizeof(struct sensors_poll_device_1));

    dev->device.common.tag = HARDWARE_DEVICE_TAG;
    dev->device.common.version = SENSORS_DEVICE_API_VERSION_1_3;
    dev->device.common.module = const_cast<hw_module_t*>(module);
    dev->device.common.close = poll__close;
    dev->device.activate = poll__activate;
    dev->device.setDelay = poll__setDelay;
    dev->device.poll = poll__poll;
    //for HAL versions >= SENSORS_DEVICE_API_VERSION_1_0
    dev->device.batch = poll__batch;
    dev->device.flush = poll__flush;
#if defined(SENSORS_DEVICE_API_VERSION_1_4)
    dev->device.inject_sensor_data = poll__inject_sensor_data;
#endif
    *device = &dev->device.common;
    status = 0;

    return status;
}


int main()
{
    struct hw_module_t module;
    char id;
    struct hw_device_t* p_hw_device_t;
    sensors_poll_context_t *dev;
    int i;
#define MAX_EVENTS_READ 26
    sensors_event_t events[MAX_EVENTS_READ];
    int msg_cnt = 0;

    open_sensors(&module, &id, &p_hw_device_t);

    for (i = 0; i < sensorsNum; ++i) {
        printf("sSensorList[%d].name = %s, handle = %d \n",
                i, sSensorList[i].name, sSensorList[i].handle);
    }
    printf("sensorsNum = %d \n", sensorsNum);

    dev = (sensors_poll_context_t *)p_hw_device_t;
    for (i = 0; i < sensorsNum; ++i) {
        dev->device.activate((sensors_poll_device_t *)dev, sSensorList[i].handle, 0);
    }

    for (i = 0; i < sensorsNum; ++i) {
        dev->device.batch((sensors_poll_device_1 *)dev, sSensorList[i].handle, 0, 5000000, 0);
        dev->device.activate((sensors_poll_device_t *)dev, sSensorList[i].handle, 1);
    }

    while(1)
    {
        msg_cnt = dev->device.poll((sensors_poll_device_t *)dev, events, MAX_EVENTS_READ);
        for (i = 0; i < msg_cnt; ++i) {

            if(META_DATA_VERSION == events[i].version &&
                    (int)0xFFFFFFFF == events[i].meta_data.sensor)
            {
                printf("ending polling...\n");
                goto END;
            }else{
                printf("get event, sensor id = %d\n", events[i].sensor);
            }
        }

        printf("==========================\n\n");
    }

END:

    for (i = 0; i < sensorsNum; ++i) {
        dev->device.activate((sensors_poll_device_t *)dev, sSensorList[i].handle, 0);
    }

    delete(dev);

    return 0;
}
