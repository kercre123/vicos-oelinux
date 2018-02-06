#ifndef _QCMAP_GSB_H_
#define _QCMAP_GSB_H_

/*======================================================

FILE:  QCMAP_GSB.h

SERVICES:
   QCMAP GSB Class

=======================================================

  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

======================================================*/
/*======================================================
  EDIT HISTORY FOR MODULE

  Please notice that the changes are listed in reverse chronological order.
    when       who        what, where, why
  --------   ---        -------------------------------------------------------
  06/01/17   gs           Created
======================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <linux/ioctl.h>
#include "qualcomm_mobile_access_point_msgr_v01.h"
#include "ds_util.h"
#include "ds_list.h"
#include "qcmap_cm_api.h"
#include "ds_string.h"
#include "QCMAP_ConnectionManager.h"




#define GSB_IOCTL_DEVICE_NAME "/dev/gsb"

#define QCMAP_GSB_DELAY_COUNT 10000 /*10 ms or 10000 us*/


#define GSB_CONFIG_TAG "GSBConfig"
#define GSB_BOOTUP_CFG_TAG "GSBBootUpcfg"
#define GSB_ENTRIES_TAG "num_of_entries"
#define GSB_INTERFACE_ENTRY_TAG "config"
#define GSB_INTERFACE_NAME_TAG "if_name"
#define GSB_INTERFACE_TYPE_TAG "if_type"
#define IPACM_ODU_TAG "ODU"
#define GSB_BW_REQD_TAG "bw_reqd"
#define GSB_IF_HIGH_WM_TAG "if_high_wm"
#define GSB_IF_LOW_WM_TAG "if_low_wm"
#define GSB_MAX_IF_SUPPORT 1

#define QCMAP_DEFAULT_CONFIG_TEMP "/data/mobileap_cfg_tmp.xml"
#define IPACM_DEFAULT_CONFIG_TEMP "/data/ipacm_cfg_tmp.xml"


#define NET_DEV_FILE_ROOT_PATH "/sys/class/net"

#define IF_STATE_UP "up"
#define IF_STATE_DOWN "down"

#define GSB_IOC_MAGIC 0xED
#define QCMAP_DEFAULT_GSB_VAL 0


#define GSB_IOC_ADD_IF_CONFIG _IOWR(GSB_IOC_MAGIC, \
                                                0, \
                                    qcmap_msgr_gsb_config_v01 *)
#define GSB_IOC_DEL_IF_CONFIG _IOWR(GSB_IOC_MAGIC, \
                                                1, \
                                    qcmap_msgr_gsb_config_v01 *)


/*=====================================================
                Helper functions Headers
  =====================================================*/

boolean UpdateIPACMcfg(char* iface_name, boolean flag);
boolean SetGSBBootUpConfig(boolean flag);
uint8 GetGSBEntryCountFromXML(void);
uint8 GetGSBConfigFromXML(qcmap_msgr_gsb_config_v01 *conf);
int IsDuplicateEntry(char * iface_name);
boolean SetGSBConfigToXML(qcmap_msgr_gsb_config_v01 *conf);
boolean RemoveGSBConfigFromXML(char * iface_name);
int isInterfaceUP(char* if_name);
void ChangeIFState(char* if_name, char* state);
int SendMSGToGSB(qcmap_msgr_gsb_config_v01 *conf, int code);


/*=====================================================
                Class definition
  =====================================================*/
class QCMAP_GSB
{
private:
  static QCMAP_GSB *object;
  static bool flag;
  static bool GSBEnableFlag;
  QCMAP_GSB();


public:
  ~QCMAP_GSB();
  static QCMAP_GSB *Get_Instance(boolean obj_create=false);

  static boolean EnableGSB(qmi_error_type_v01 *qmi_err_num );
  static boolean DisableGSB(qmi_error_type_v01 *qmi_err_num );

  static boolean SetGSBConfig(qcmap_msgr_gsb_config_v01 *gsb_conf,
                                qmi_error_type_v01 *qmi_err_num );

  static boolean GetGSBConfig(qcmap_msgr_gsb_config_v01 *gsb_conf,
                              uint8 *num_of_entries,
                              qmi_error_type_v01 *qmi_err_num);

  static boolean DeleteGSBConfig(char* if_name,
                                 qmi_error_type_v01 *qmi_err_num );

};



#endif
