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
 * @file         sensord_hwcntl_iio.h
 * @date         "Fri Dec 11 10:40:18 2015 +0800"
 * @commit       "4498a7f"
 *
 * @brief
 *
 * @detail
 *
 */

#ifndef SENSORD_HWCNTL_IIO_H_
#define SENSORD_HWCNTL_IIO_H_

/**
 *
 * @param name
 * @param iio_dir
 * @return
 */
static inline int get_IIOnum_by_name(const char *name, const char *iio_dir)
{
#define IIO_NAME_MAXLEN 30
#define MAX_FILENAME_LEN 256
    const char *type = "iio:device";
    struct dirent *ent;
    struct dirent dirent;
    int number, numstrlen;

    FILE *nameFile;
    DIR *dp;
    char thisname[IIO_NAME_MAXLEN];
    char fname_buf[MAX_FILENAME_LEN+1];
    int ret;

    dp = opendir(iio_dir);
    if (NULL == dp)
    {
        return -ENODEV;
    }

    while (!readdir_r(dp, &dirent, &ent) && NULL != ent)
    {
        if (0 == strcmp(ent->d_name, ".") ||
                0 == strcmp(ent->d_name, "..") ||
                strlen(ent->d_name) <= strlen(type) ||
                0 != strncmp(ent->d_name, type, strlen(type)))
        {
            /*filter impossible dir names*/
            continue;
        }

        numstrlen = sscanf(ent->d_name + strlen(type), "%d", &number);

        /* verify the next character is not a colon */
        if(0 == strncmp(ent->d_name + strlen(type) + numstrlen, ":", 1))
        {
            continue;
        }

        snprintf(fname_buf, MAX_FILENAME_LEN, "%s%s%d/name", iio_dir, type, number);

        nameFile = fopen(fname_buf, "r");
        if (!nameFile)
        {
            continue;
        }

        ret = fscanf(nameFile, "%s", thisname);
        if(ret <= 0)
        {
            fclose(nameFile);
            break;
        }

        if (0 == strcmp(name, thisname))
        {
            fclose(nameFile);
            closedir(dp);
            return number;
        }

        fclose(nameFile);
    }

    closedir(dp);
    return -ENODEV;
}

#define MAX_FILENAME_LEN 256

static inline int wr_sysfs_twoint(const char *filename, char *basedir, int val1, int val2)
{
    FILE *fp;
    char fname_buf[MAX_FILENAME_LEN+1];

    snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

    fp = fopen(fname_buf, "w");
    if (NULL == fp)
    {
        return -errno;
    }

    fprintf(fp, "%d %d", val1, val2);
    fclose(fp);

    return 0;
}

static inline  int wr_sysfs_oneint(const char *filename, char *basedir, int val)
{
    FILE *fp;
    char fname_buf[MAX_FILENAME_LEN+1];

    snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

    fp = fopen(fname_buf, "w");
    if (NULL == fp)
    {
        return -errno;
    }

    fprintf(fp, "%d", val);
    fclose(fp);

    return 0;
}


static inline  int wr_sysfs_str(const char *filename, char *basedir, const char *str)
{
    FILE *fp;
    char fname_buf[MAX_FILENAME_LEN+1];

    snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

    fp = fopen(fname_buf, "w");
    if (NULL == fp)
    {
        return -errno;
    }

    fprintf(fp, "%s", str);
    fclose(fp);

    return 0;
}


static inline int rd_sysfs_oneint(const char *filename, char *basedir, int *pval)
{
    FILE *fp;
    char fname_buf[MAX_FILENAME_LEN+1];
    int ret;

    snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", basedir, filename);

    fp = fopen(fname_buf, "r");
    if (NULL == fp)
    {
        return -errno;
    }

    ret = fscanf(fp, "%d\n", pval);
    fclose(fp);

    if(ret <= 0){
        return -errno;
    }

    return 0;
}


#endif /* SENSORD_HWCNTL_IIO_H_ */
