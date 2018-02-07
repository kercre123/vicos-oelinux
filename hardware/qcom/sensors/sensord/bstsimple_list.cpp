/*!
 * @section LICENSE
 *
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *------------------------------------------------------------------------------
 *  Disclaimer
 *
 * Common: Bosch Sensortec products are developed for the consumer goods
 * industry. They may only be used within the parameters of the respective valid
 * product data sheet.  Bosch Sensortec products are provided with the express
 * understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive
 * systems or any system or device that may lead to bodily harm or property
 * damage if the system or device malfunctions. In addition, Bosch Sensortec
 * products are not fit for use in products which interact with motor vehicle
 * systems.  The resale and/or use of products are at the purchaser's own risk
 * and his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims,
 * including any claims for incidental, or consequential damages, arising from
 * any product use not covered by the parameters of the respective valid product
 * data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec
 * for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec without
 * delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
 * from the valid technical specifications of the product series. They are
 * therefore not intended or fit for resale to third parties or for use in end
 * products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series.
 * Bosch Sensortec assumes no liability for the use of engineering samples. By
 * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
 * Sensortec from all claims arising from the use of engineering samples.
 *
 * Special: This software module (hereinafter called "Software") and any
 * information on application-sheets (hereinafter called "Information") is
 * provided free of charge for the sole purpose to support your application
 * work. The Software and Information is subject to the following terms and
 * conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch
 * Sensortec products by personnel who have special experience and training. Do
 * not use this Software if you do not have the proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or
 * implied warranties, including without limitation, the implied warranties of
 * merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for
 * the functional impairment of this Software in terms of fitness, performance
 * and safety. Bosch Sensortec and their representatives and agents shall not be
 * liable for any direct or indirect damages or injury, except as otherwise
 * stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use.
 *
 * @file         bstsimple_list.cpp
 * @date         "Tue Sep 15 02:36:56 2015 -0400"
 * @commit       "1758191"
 *
 * @brief
 *
 * @detail
 *
 */

#include "bstsimple_list.h"
#include "sensord_pltf.h"

#define DEFAULT_LIST_LEN 128

BstSimpleList::BstSimpleList()
{
    head = NULL;
    tail = NULL;
    list_len = 0;
    set_uplimit(DEFAULT_LIST_LEN);

    return;
}

BstSimpleList::~BstSimpleList()
{
    list_clean();
    return;
}

void BstSimpleList::set_uplimit(uint32_t limit)
{
    if (0 == limit)
    {
        //at least 1 node can be stored
        // to log something...
        limit = 1;
    }
    uplimit = limit;
}

int BstSimpleList::list_add_rear(void *pdata)
{
    struct list_node *nod;
    void *del;
    int ret = 0;

    nod = (struct list_node *) malloc(sizeof(struct list_node));
    if (NULL == nod)
    {
        return -1;
    }

    if (list_len == uplimit)
    {
        ret = -2;
        PERR("list buffer is full, drop the oldest data");
        list_get_headdata(&del);
        free(del);
    }

    nod->p_data = pdata;
    nod->next = NULL;
    if (NULL == head)
    {
        /*to be convenient, when running,
         tail point is allowed to have obsolete value.
         so use head point to judge if empty
         */
        //first node added
        tail = nod;
        head = nod;
    }
    else
    {
        tail->next = nod;
        tail = nod;
    }
    list_len++;

    return ret;
}

void BstSimpleList::list_get_headdata(void **ppdata)
{
    struct list_node *cur;

    if (0 == list_len)
    {
        *ppdata = NULL;
        return;
    }

    *ppdata = head->p_data;
    cur = head;
    head = head->next;
    list_len--;

    free(cur);

    return;
}

int BstSimpleList::list_mount_rear(BstSimpleList *list_for_mnt)
{
    void *pdata = NULL;
    int ret = 0;

    if (NULL == list_for_mnt || 0 == list_for_mnt->list_len)
    {
        return 0;
    }

    if (NULL == head)
    {
        /*to be convenient, when running,
         tail point is allowed to have obsolete value.
         so use head point to judge if empty
         */
        //the destined list is yet empty
        head = list_for_mnt->head;
        tail = list_for_mnt->tail;
    }
    else
    {
        tail->next = list_for_mnt->head;
        tail = list_for_mnt->tail;
    }
    list_len += list_for_mnt->list_len;

    list_for_mnt->head = NULL;
    list_for_mnt->tail = NULL;
    list_for_mnt->list_len = 0;

    //truncate to uplimit
    while (list_len > uplimit)
    {
        ret = -1;
        PERR("add too much, drop the oldest data");
        list_get_headdata(&pdata);
        free(pdata);
    }

    return ret;
}

int BstSimpleList::list_clean()
{
    void *pdata = NULL;

    while (list_len)
    {
        list_get_headdata(&pdata);
        free(pdata);
    }

    return 0;
}

