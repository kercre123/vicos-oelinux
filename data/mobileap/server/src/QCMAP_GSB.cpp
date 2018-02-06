/*====================================================

FILE:  QCMAP_GSB.cpp

SERVICES:
   QCMAP GSB Specific Implementation

=====================================================

  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

=====================================================*/
/*======================================================
  EDIT HISTORY FOR MODULE

  Please notice that the changes are listed in reverse chronological order.

  when       who        what, where, why
  --------   ---        -------------------------------------------------------
  06/01/17   gs         Created

======================================================*/
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ds_string.h"
#include "ds_util.h"
#include "QCMAP_GSB.h"
#include "QCMAP_ConnectionManager.h"
#include "qcmap_cm_api.h"

bool QCMAP_GSB::flag = false;
QCMAP_GSB* QCMAP_GSB::object=NULL;
bool QCMAP_GSB::GSBEnableFlag = false;

/*=====================================================
                             Helper functions
  =====================================================*/


boolean UpdateIPACMcfg(char* iface_name, boolean flag)
{
  pugi::xml_document xml_file;
  pugi::xml_node parent, root, child;
  char command[MAX_COMMAND_STR_LEN];


  if(!xml_file.load_file(IPA_XML_PATH))
  {
    LOG_MSG_ERROR("Unable to load IPACM XML file.",0,0,0);
    return false;
  }

  root = xml_file.child(System_TAG).child(IPACM_TAG).child(IPACMIface_TAG);
  for (child = root.first_child(); child; child = child.next_sibling())
  {
    if (!strncmp(child.child(Name_TAG).child_value(),
                     iface_name, strlen(iface_name)))
    {
      if (flag)
      {
        child.child(Category_TAG).text() = IPACM_ODU_TAG;
        child.remove_child(WLANMode_TAG);
        xml_file.save_file(IPACM_DEFAULT_CONFIG_TEMP);
        snprintf( command, MAX_COMMAND_STR_LEN,"fsync -d %s",IPACM_DEFAULT_CONFIG_TEMP);
        ds_system_call(command, strlen(command));
        snprintf( command, MAX_COMMAND_STR_LEN,"mv %s %s ",IPACM_DEFAULT_CONFIG_TEMP,
                                          IPA_XML_PATH);
        ds_system_call(command, strlen(command));
        LOG_MSG_INFO1("IPACM cfg file changed successfully for %s", iface_name, 0, 0);
        return true;
      }
    }
  }

  LOG_MSG_ERROR( "Failed to find iface %s",iface_name,0,0 );
  return false;
}

boolean SetGSBBootUpConfig(boolean flag)
{
  QCMAP_ConnectionManager* QcMapMgr=QCMAP_ConnectionManager::Get_Instance(NULL,false);
  pugi::xml_document xml_file;
  pugi::xml_node root, entry_node;
  char data[MAX_STRING_LENGTH] = { 0 };
  char command[MAX_COMMAND_STR_LEN];

  if (!QcMapMgr)
  {
    return false;
  }

  if (!xml_file.load_file(QCMAP_DEFAULT_CONFIG))
  {
    LOG_MSG_ERROR("error loading XML file",0,0,0);
    return false;
  }

  LOG_MSG_INFO1( "Setting GSB bootup config=%d",flag, 0,0 );
  root = xml_file.child(System_TAG).child(MobileAPCfg_TAG).child(GSB_CONFIG_TAG);
  entry_node = root.child(GSB_BOOTUP_CFG_TAG);

  snprintf(data, sizeof(data), "%d", flag);
  entry_node.first_child().set_value(data);

  xml_file.save_file(QCMAP_DEFAULT_CONFIG_TEMP);
  snprintf( command, MAX_COMMAND_STR_LEN,"fsync -d %s",QCMAP_DEFAULT_CONFIG_TEMP);
  ds_system_call(command, strlen(command));
  snprintf( command, MAX_COMMAND_STR_LEN,"mv %s %s ",QCMAP_DEFAULT_CONFIG_TEMP,
                                          QCMAP_DEFAULT_CONFIG);
  ds_system_call(command, strlen(command));
  return true;
}

uint8 GetGSBEntryCountFromXML(void)
{
  uint8 entries = 0;
  pugi::xml_document xml_file;


  if (!xml_file.load_file(QCMAP_DEFAULT_CONFIG))
  {
    LOG_MSG_ERROR("error loading XML file",0,0,0);
    return 0;
  }

  entries = atoi(xml_file.child(System_TAG).child(MobileAPCfg_TAG).
                 child(GSB_CONFIG_TAG).child(GSB_ENTRIES_TAG).first_child().value());
  LOG_MSG_INFO1("entries %d", entries,0,0);

  return entries;
}

uint8 GetGSBConfigFromXML(qcmap_msgr_gsb_config_v01 *conf)
{
  uint8 entries = 0;
  uint8 i = 0;
  pugi::xml_node root, child;
  pugi::xml_node conf_root, conf_child;
  pugi::xml_node if_root;
  pugi::xml_document xml_file;


  if (!xml_file.load_file(QCMAP_DEFAULT_CONFIG))
  {
    LOG_MSG_ERROR("error loading XML file",0,0,0);
    return 0;
  }

  root = xml_file.child("system").first_child();

  if (root)
  {
    for (child = root.first_child(); child; child = child.next_sibling())
    {
      if (strncmp(child.name(), GSB_CONFIG_TAG,strlen(child.name())) == 0 )
      {
        conf_root = child;

        for (child = conf_root.first_child(); child; child = child.next_sibling())
        {
          if (strncmp(child.name(), GSB_ENTRIES_TAG,strlen(child.name())) == 0 )
          {
            LOG_MSG_INFO1("entries %d", atoi(child.first_child().value()),0,0);
            entries = atoi(child.first_child().value());
            if (entries == 0) break;
          }
          if (strncmp(child.name(), GSB_INTERFACE_ENTRY_TAG,strlen(child.name())) == 0 )
          {
            if_root = child;
            for (conf_child = if_root.first_child(); conf_child; conf_child = conf_child.next_sibling())
            {
              if (!strncmp(conf_child.name(), GSB_INTERFACE_NAME_TAG,strlen(conf_child.name())))
              {
                LOG_MSG_INFO1("if_name: %s", conf_child.first_child().value(),0,0);
                strlcpy(conf[i].if_name, conf_child.first_child().value(),
                                                  QCMAP_MAX_IFACE_NAME_SIZE_V01);
              }

              if (!strncmp(conf_child.name(), GSB_INTERFACE_TYPE_TAG,strlen(conf_child.name())))
              {
                LOG_MSG_INFO1("if type %d", atoi(conf_child.first_child().value()),0,0);
                conf[i].if_type = (qcmap_msgr_interface_type_enum_v01)atoi(conf_child.first_child().value());
              }


              if (!strncmp(conf_child.name(), GSB_BW_REQD_TAG,strlen(conf_child.name())))
              {
                LOG_MSG_INFO1("bw reqd %d", atoi(conf_child.first_child().value()),0,0);
                conf[i].bw_reqd_in_mb = atoi(conf_child.first_child().value());
              }


              if (!strncmp(conf_child.name(), GSB_IF_HIGH_WM_TAG,strlen(conf_child.name())))
              {
                LOG_MSG_INFO1("high wm %d", atoi(conf_child.first_child().value()),0,0);
                conf[i].if_high_watermark = atoi(conf_child.first_child().value());
              }


              if (!strncmp(conf_child.name(), GSB_IF_LOW_WM_TAG, strlen(conf_child.name())))
              {
                LOG_MSG_INFO1("low wm %d", atoi(conf_child.first_child().value()),0,0);
                conf[i].if_low_watermark = atoi(conf_child.first_child().value());
              }
            }
            i++;
          }
        }
      }
    }
  }

  if (i != entries)
  {
    LOG_MSG_ERROR("config is not correct",0,0,0);
  }
  return entries;
}

int IsDuplicateEntry(char * iface_name)
{
  uint8 entries = 0;
  pugi::xml_node root, child;
  pugi::xml_node conf_child;
  pugi::xml_node if_root;
  pugi::xml_document xml_file;



  if (iface_name == "" || iface_name == NULL)
  {
    LOG_MSG_ERROR("NULL ifname passed", 0, 0, 0);
    return -1;
  }

  if (!xml_file.load_file(QCMAP_DEFAULT_CONFIG))
  {
    LOG_MSG_INFO1("error loading XML file",0,0,0);
    return -1;
  }

  root = xml_file.child(System_TAG).child(MobileAPCfg_TAG).child(GSB_CONFIG_TAG);

  if (root)
  {
    for (child = root.first_child(); child; child = child.next_sibling())
    {
      if (!strncmp(child.name(), GSB_ENTRIES_TAG, strlen(child.name())))
      {
        entries = atoi(child.first_child().value());
        if (entries == 0)
        {
          LOG_MSG_ERROR("No entry present",0,0,0);
          return 0;
        }
      }

      if (!strncmp(child.name(), GSB_INTERFACE_ENTRY_TAG,strlen(child.name())))
      {
        if_root = child;

        for (conf_child = if_root.first_child(); conf_child; conf_child = conf_child.next_sibling())
        {
          if (!strncmp(conf_child.name(), GSB_INTERFACE_NAME_TAG,strlen(conf_child.name())))
          {
            if ((strncmp(conf_child.first_child().value(), iface_name,
                                    strlen(conf_child.first_child().value())) == 0))
            {
              LOG_MSG_INFO1("duplicate entry for if_name %s found, requested: %s",
                                    conf_child.first_child().value(),iface_name,0);
              return 1;
            }
          }
        }
      }
    }
  }
  else
  {
    LOG_MSG_ERROR("Error in obtaining GSB node in cfg file", 0, 0, 0);
    return -1;
  }
/*should not reach here*/
  return -1;
}

boolean SetGSBConfigToXML(qcmap_msgr_gsb_config_v01 *conf)
{
  uint8 curr_entries = 0;
  uint8 new_entry = 0;
  uint8 i = 0;
  pugi::xml_node root, child, subchild;
  pugi::xml_document xml_file;
  char data[MAX_STRING_LENGTH] = { 0 };
  char command[MAX_COMMAND_STR_LEN];


  if (!xml_file.load_file(QCMAP_DEFAULT_CONFIG))
  {
    LOG_MSG_INFO1("error loading XML file",0,0,0);
    return 0;
  }

  root = xml_file.child(System_TAG).child(MobileAPCfg_TAG).child(GSB_CONFIG_TAG);

  if (root)
  {
    curr_entries = atoi(root.child(GSB_ENTRIES_TAG).first_child().value());
    LOG_MSG_INFO1("curr entries %d", curr_entries,0,0);
  }

  child = root.append_child(GSB_INTERFACE_ENTRY_TAG);

  subchild = child.append_child(GSB_INTERFACE_NAME_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(conf->if_name);

  snprintf(data, sizeof(data), "%d", conf->if_type);
  subchild = child.append_child(GSB_INTERFACE_TYPE_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(data);

  snprintf(data, sizeof(data), "%d", conf->bw_reqd_in_mb);
  subchild = child.append_child(GSB_BW_REQD_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(data);

  snprintf(data, sizeof(data), "%d", conf->if_high_watermark);
  subchild = child.append_child(GSB_IF_HIGH_WM_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(data);

  snprintf(data, sizeof(data), "%d", conf->if_low_watermark);
  subchild = child.append_child(GSB_IF_LOW_WM_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(data);


  curr_entries++;
  snprintf(data, sizeof(data), "%d", curr_entries);
  root.child(GSB_ENTRIES_TAG).first_child().set_value(data);

  new_entry = atoi(root.child(GSB_ENTRIES_TAG).first_child().value());
  LOG_MSG_INFO1("new entriess %d", new_entry,0,0);


  xml_file.save_file(QCMAP_DEFAULT_CONFIG_TEMP);
  snprintf( command, MAX_COMMAND_STR_LEN,"fsync -d %s",QCMAP_DEFAULT_CONFIG_TEMP);
  ds_system_call(command, strlen(command));
  snprintf( command, MAX_COMMAND_STR_LEN,"mv %s %s ",QCMAP_DEFAULT_CONFIG_TEMP,
                                          QCMAP_DEFAULT_CONFIG);
  ds_system_call(command, strlen(command));

  return 1;
}

boolean RemoveGSBConfigFromXML(char * iface_name)
{
  uint8 entries = 0;
  uint8 i = 0;
  pugi::xml_node root, child;
  pugi::xml_node conf_root, conf_child;
  pugi::xml_node if_root;
  pugi::xml_document xml_file;
  boolean if_found = false;
  char command[MAX_COMMAND_STR_LEN];
  char data[MAX_STRING_LENGTH] = { 0 };


  if (iface_name == "" || iface_name == NULL)
  {
    return 0;
  }

  if (!xml_file.load_file(QCMAP_DEFAULT_CONFIG))
  {
    LOG_MSG_ERROR("error loading XML file",0,0,0);
    return 0;
  }

  root = xml_file.child("system").first_child();

  if (root)
  {
    for (child = root.first_child(); child; child = child.next_sibling())
    {
      if (!strncmp(child.name(), GSB_CONFIG_TAG,strlen(child.name())))
      {
        conf_root = child;

        for (child = conf_root.first_child(); child; child = child.next_sibling())
        {
          if (!strncmp(child.name(), GSB_ENTRIES_TAG, strlen(child.name())))
          {
            entries = atoi(child.first_child().value());
            if (entries == 0)
            {
              LOG_MSG_ERROR("No entry to delete",0,0,0);
              return 0;
            }
            else
            {
              LOG_MSG_INFO1("Total entries available %d",entries,0,0);
            }
          }

          if (!strncmp(child.name(), GSB_INTERFACE_ENTRY_TAG,strlen(child.name())))
          {
            if_root = child;

            for (conf_child = if_root.first_child(); conf_child; conf_child = conf_child.next_sibling())
            {
              if (!strncmp(conf_child.name(), GSB_INTERFACE_NAME_TAG,strlen(conf_child.name())))
              {
                if ((strncmp(conf_child.first_child().value(), iface_name,
                                        strlen(conf_child.first_child().value())) == 0))
                {
                  LOG_MSG_INFO1("removing if_name %s, requested: %s",
                                        conf_child.first_child().value(),iface_name,0);
                  if_found = true;
                  if_root.parent().remove_child(if_root);
                }
              }
            }
          }
        }
      }
    }
  }

  if (if_found)
  {
    entries--;
    LOG_MSG_INFO1("Node with %s name is removed successfully, entry %d",iface_name,entries,0);
    snprintf(data, sizeof(data), "%d", entries);
    xml_file.child(System_TAG).child(MobileAPCfg_TAG).
    child(GSB_CONFIG_TAG).child(GSB_ENTRIES_TAG).first_child().set_value(data);
    xml_file.save_file(QCMAP_DEFAULT_CONFIG_TEMP);
    snprintf( command, MAX_COMMAND_STR_LEN,"fsync -d %s",QCMAP_DEFAULT_CONFIG_TEMP);
    ds_system_call(command, strlen(command));
    snprintf( command, MAX_COMMAND_STR_LEN,"mv %s %s ",QCMAP_DEFAULT_CONFIG_TEMP,
                                            QCMAP_DEFAULT_CONFIG);
    ds_system_call(command, strlen(command));
    return 1;
  }
  else
  {
    LOG_MSG_ERROR("Node with %s name is not found",iface_name,0,0);
    return 0;
  }
}

int isInterfaceUP(char* if_name)
{
  char buf[MAX_COMMAND_STR_LEN];
  char command[MAX_COMMAND_STR_LEN];
  char tmp[MAX_COMMAND_STR_LEN]={0};
  FILE *stream = NULL;


  snprintf( command, MAX_COMMAND_STR_LEN,NET_DEV_FILE_ROOT_PATH);
  snprintf(tmp, MAX_COMMAND_STR_LEN, "/%s/operstate", if_name);
  strlcat(command, tmp, MAX_COMMAND_STR_LEN);

  LOG_MSG_INFO1("file path %s", command,0,0);

  stream = fopen(command, "r");
  if(NULL == stream)
  {
    LOG_MSG_ERROR("Failed to open if file",0,0,0);
    return -1;
  }

  fread(buf, sizeof(char), MAX_COMMAND_STR_LEN, stream);
  fclose(stream);

  if ((strncmp(buf, IF_STATE_UP, strlen(IF_STATE_UP)) == 0))
  {
    LOG_MSG_INFO1("if %s state is up, buf in %s state",if_name,buf,0);
    return 1;
  }
  else
  {
    LOG_MSG_INFO1("if %s state is down, buf in %s state",if_name,buf,0);
    return 0;
  }

}

void ChangeIFState(char* if_name, char* state)
{
  char command[MAX_COMMAND_STR_LEN];
  snprintf( command, MAX_COMMAND_STR_LEN,"ifconfig %s %s", if_name, state);
  ds_system_call( command, strlen(command));
}

int SendMSGToGSB(qcmap_msgr_gsb_config_v01 *conf, int code)
{
  int ret = -1;
  int gsb_ioctl_fd = open(GSB_IOCTL_DEVICE_NAME, O_RDWR);
  if(gsb_ioctl_fd < 0)
  {
    LOG_MSG_INFO1("Failed to open GSB device node: %d",errno, 0, 0);
    return -1;
  }
  ret = ioctl(gsb_ioctl_fd, code, conf);
  if (ret != 0)
  {
    LOG_MSG_INFO1("Failed to send code %d to GSB device node error %d",code, errno, 0);
    close(gsb_ioctl_fd);
    return -1;
  }
  close(gsb_ioctl_fd);
  return 1;
}




/*=====================================================
              Class Implementation
  =====================================================*/
/*=====================================================
  FUNCTION Constructor
======================================================*/
/*!
@brief
  Initializes GSB OBject.

@parameters
none

@return
  none

@note
- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/
QCMAP_GSB::QCMAP_GSB()
{

}

/*======================================================
  FUNCTION Destructor
======================================================*/
/*!
@brief
  Destroyes the GSB Object.

@parameters
none

@return
  None

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=====================================================*/


QCMAP_GSB::~QCMAP_GSB()
{
  QCMAP_GSB* QcMapGSBObj = QCMAP_GSB::Get_Instance(false);
  flag=false;
  if (QcMapGSBObj != NULL)
  {
    delete QcMapGSBObj;
  }
  object = NULL;
}


/*=====================================================
  FUNCTION Get_Instance
======================================================*/
/*!
@brief
  Gets and return instance of class QCMAP_GSB

@parameters
  obj_create - flag to check if object/instance is already exist

@return
  object -  object created

@note
- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

QCMAP_GSB* QCMAP_GSB::Get_Instance(boolean obj_create)
{
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  if(!flag && obj_create)
  {
    LOG_MSG_INFO1("Creating object : GSB",0, 0, 0);
    object = new QCMAP_GSB();
    flag = true;
    return object;
  }
  else
  {
    return object;
  }
}

/*==========================================================
 FUNCTION    EnableGSB
===========================================================*/
/*!
@brief
  Enables GSB

@parameters

@return
  1 - on success
  0 - on failure

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
boolean QCMAP_GSB::EnableGSB
(
  qmi_error_type_v01 *qmi_err_num
)
{
  char command[MAX_COMMAND_STR_LEN];
  QCMAP_ConnectionManager* QcMapMgr=QCMAP_ConnectionManager::Get_Instance(NULL,false);
  char Kernel_ver[KERNEL_VERSION_LENGTH];
  qcmap_msgr_gsb_config_v01 config[QCMAP_MSGR_MAX_IF_SUPPORTED_V01];
  uint8 entries = 0;
  int ret = 0;
  boolean if_stopped = false;

  if (QCMAP_GSB::GSBEnableFlag)
  {
    LOG_MSG_ERROR("GSB already enabled", 0, 0, 0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  if ( false == QcMapMgr->GetKernelVer(Kernel_ver) )
  {
    LOG_MSG_ERROR("Unable to get the kernel version info", 0, 0, 0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  //read the configuration
  entries = GetGSBConfigFromXML(config);
  //if no configuration entry found then return
  if (!entries)
  {
    LOG_MSG_ERROR("No configuration found",0,0,0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  // if there is even a single config then load the module using insmod
  snprintf(command,MAX_COMMAND_STR_LEN,
             "insmod /usr/lib/modules/%s/extra/gsb.ko",Kernel_ver);
  ds_system_call(command, strlen(command));

  usleep(QCMAP_GSB_DELAY_COUNT * 50);

  //stop all the  interfaces who are configured to work with GSB
  // if interfaces are not up then  skip
  for (int j = 0 ; j < entries; j++)
  {
    if_stopped = false;
    //Pass the configuration to GSB via IOCTL
    if(SendMSGToGSB(&config[j], GSB_IOC_ADD_IF_CONFIG)<0)
    {
      LOG_MSG_ERROR("Failed to send msg to GSB", 0,0,0);
      *qmi_err_num = QMI_ERR_INTERNAL_V01;
      return false;
    }
    usleep(QCMAP_GSB_DELAY_COUNT * 50);

    ret = isInterfaceUP(config[j].if_name);
    if (ret == 1)
    {
      //stop IF
      ChangeIFState(config[j].if_name, IF_STATE_DOWN);
      if_stopped = true;
      snprintf(command, MAX_COMMAND_STR_LEN, "echo [GSB]: if %s disabled > /dev/kmsg",config[j].if_name);
      ds_system_call(command, strlen(command));
    }

    if(if_stopped)
    {
      ChangeIFState(config[j].if_name, IF_STATE_UP);
      snprintf(command, MAX_COMMAND_STR_LEN, "echo [GSB]: if %s enabled > /dev/kmsg",config[j].if_name);
      ds_system_call(command, strlen(command));
    }
  }

  //set enable flag for gsb
  QCMAP_GSB::GSBEnableFlag = true;
  if (!QcMapMgr->cfg.bootup_config.enable_gsb_at_bootup)
  {
    if(!SetGSBBootUpConfig(true))
    {
      LOG_MSG_ERROR("Error enabling bootup config", 0,0,0);
      *qmi_err_num = QMI_ERR_INTERNAL_V01;
      return false;
    }
  }

  return 1;
}

/*==========================================================
 FUNCTION    DisableGSB
===========================================================*/
/*!
@brief
  Disables GSB

@parameters

@return
  1 - on success
  0 - on failure

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
boolean QCMAP_GSB::DisableGSB
(
  qmi_error_type_v01 *qmi_err_num
)
{
  qcmap_msgr_gsb_config_v01 config[QCMAP_MSGR_MAX_IF_SUPPORTED_V01];
  uint8 entries = 0;
  boolean if_stopped[QCMAP_MSGR_MAX_IF_SUPPORTED_V01];
  int ret = -1;

  memset(if_stopped,0, QCMAP_MSGR_MAX_IF_SUPPORTED_V01);

  //if not enabled return 0
  if (!QCMAP_GSB::GSBEnableFlag)
  {
    LOG_MSG_ERROR("GSB not enabled",0,0,0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  // if enabled, then read configuration
  entries = GetGSBConfigFromXML(config);
  //if no configuration entry found then return
  if (!entries)
  {
    LOG_MSG_ERROR("No configuration found",0,0,0);
    //unload GSB module
    ds_system_call("rmmod gsb",
            strlen("rmmod gsb"));
    QCMAP_GSB::GSBEnableFlag = false;
    return true;
  }
  else
  {
    //Stop all the enabled IF (the one which are configured  only)
    //If not enabled then move to next step
    for (int j = 0 ; j < entries; j++)
    {
      ret = isInterfaceUP(config[j].if_name);
      if (ret == 1)
      {
        //stop IF
        ChangeIFState(config[j].if_name, IF_STATE_DOWN);
        if_stopped [j]= true;
      }
    }
   //unload GSB module
    ds_system_call("rmmod gsb",
            strlen("rmmod gsb"));
    usleep(QCMAP_GSB_DELAY_COUNT * 50);


    //restart the interfaces if stopped.
    for (int j = 0 ; j < entries; j++)
    {
      if (if_stopped[j])
      {
        ChangeIFState(config[j].if_name, IF_STATE_UP);
      }
    }
  }

  // clear enable flag.
  QCMAP_GSB::GSBEnableFlag = false;
  if(!SetGSBBootUpConfig(false))
  {
    LOG_MSG_ERROR("Error disabling bootup config", 0,0,0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }
  return true;
}

/*==========================================================
 FUNCTION    DeleteGSBConfig
===========================================================*/
/*!
@brief
  Delete GSB Config

@parameters

@return
  1 - on success
  0 - on failure

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
boolean QCMAP_GSB::DeleteGSBConfig
(
  char* if_name,
  qmi_error_type_v01 *qmi_err_num
)
{
  qcmap_msgr_gsb_config_v01 gsb_conf;
  int ret = 0;
  boolean if_stopped = false;
  char command[MAX_COMMAND_STR_LEN];
  uint8 *num_of_entries = 0;


  LOG_MSG_INFO1("removing %s IFACE from GSB",if_name,0,0);

  /* Restore the previous xml file, command will fail if _bak file does not exist */
  /*if file dont exisit then following commads will fail silently*/
  snprintf(command, MAX_COMMAND_STR_LEN,"mv %s_bak %s",IPA_XML_PATH, IPA_XML_PATH);
  ds_system_call(command, strlen(command));


  if (!RemoveGSBConfigFromXML(if_name))
  {
    LOG_MSG_ERROR("Error removing config from XML",0,0,0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  strlcpy(gsb_conf.if_name, if_name, QCMAP_MAX_IFACE_NAME_SIZE_V01);

  //if GSB is enabled, it can be assumed that its cache is updated
  //with the configuration. So we need ot remove the IF from GSB cache too.
  if (QCMAP_GSB::GSBEnableFlag)
  {
    if_stopped = false;
    ret = isInterfaceUP(if_name);
    if (ret == 1)
    {
      //stop IF
      ChangeIFState(if_name, IF_STATE_DOWN);
      if_stopped = true;
    }
    usleep(QCMAP_GSB_DELAY_COUNT * 10);
    //Pass the configuration to GSB via IOCTL.
    //We are only interested in if_name in this case.
    if(SendMSGToGSB(&gsb_conf, GSB_IOC_DEL_IF_CONFIG)<0)
    {
      LOG_MSG_ERROR("Failed to send msg to GSB", 0,0,0);
      *qmi_err_num = QMI_ERR_INTERNAL_V01;
      return false;
    }
    else
    {
      LOG_MSG_INFO1("sent ioctl to gsb\n",0,0,0);
    }
    /* this specific Ioctl requires cleaning so adding some time*/
    usleep(QCMAP_GSB_DELAY_COUNT * 100);

    num_of_entries = GetGSBEntryCountFromXML();
    if (num_of_entries == 0)
    {
      LOG_MSG_INFO1("No entry found , Unloading GSB",0,0,0);
      //unload GSB module
      ds_system_call("rmmod gsb",
              strlen("rmmod gsb"));
      usleep(QCMAP_GSB_DELAY_COUNT * 50);

      QCMAP_GSB::GSBEnableFlag = false;
      /*no reason to have GSB enabled at bootup either*/
      LOG_MSG_INFO1("Disabling GSB boot up configuration",0,0,0);
      if(!SetGSBBootUpConfig(false))
      {
        LOG_MSG_ERROR("Error disabling bootup config ",0,0,0);
        *qmi_err_num = QMI_ERR_INTERNAL_V01;
        return false;
      }
    }

    // restart stopped interfaces, if they were stopped
    if(if_stopped)
    {
      ChangeIFState(if_name, IF_STATE_UP);
    }
  }
  else
  {
    LOG_MSG_INFO1("cannot send msg to gsb as it is not enabled",0,0,0);
  }

  return true;
}

/*==========================================================
 FUNCTION    SetGSBConfig
===========================================================*/
/*!
@brief
  Sets GSB Configuration

@parameters

@return
  1 - on success
  0 - on failure

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
boolean QCMAP_GSB::SetGSBConfig
(
  qcmap_msgr_gsb_config_v01 *gsb_conf,
  qmi_error_type_v01 *qmi_err_num
)
{
  char command[MAX_COMMAND_STR_LEN];
  uint8 *num_of_entries = 0;

  num_of_entries = GetGSBEntryCountFromXML();
  if ((num_of_entries+1) > GSB_MAX_IF_SUPPORT)
  {
    LOG_MSG_ERROR("Cannot add %s to GSB config. Max iface reached",gsb_conf->if_name,0,0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  if (IsDuplicateEntry(gsb_conf->if_name) == 0)
  {
    if (gsb_conf->if_type == QCMAP_MSGR_INTERFACE_TYPE_WLAN_V01)
    {
      snprintf(command, MAX_COMMAND_STR_LEN,"cp %s %s_bak",IPA_XML_PATH, IPA_XML_PATH);
      ds_system_call(command, strlen(command));
      if (!UpdateIPACMcfg(gsb_conf->if_name, true))
      {
        LOG_MSG_ERROR("Error setting config to IPA cfg XML for iface %s",gsb_conf->if_name,0,0);
        *qmi_err_num = QMI_ERR_INTERNAL_V01;
        return false;
      }
    }

    if (!SetGSBConfigToXML(gsb_conf))
    {
      LOG_MSG_ERROR("Error setting config to XML",0,0,0);
      *qmi_err_num = QMI_ERR_INTERNAL_V01;
      return false;
    }
  }
  else
  {
    if (IsDuplicateEntry(gsb_conf->if_name) == 1)
    {
      LOG_MSG_ERROR("Entry already present", 0, 0, 0);
    }
    else
    {
      LOG_MSG_ERROR("Problem accessing gsb config", 0, 0, 0);
    }

    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  return true;
}

/*==========================================================
 FUNCTION    GetGSBConfig
===========================================================*/
/*!
@brief
  Gets GSB Configuration

@parameters

@return
  1 - on success
  0 - on failure

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
boolean QCMAP_GSB::GetGSBConfig
(
  qcmap_msgr_gsb_config_v01 *gsb_conf,
  uint8 *num_of_entries,
  qmi_error_type_v01 *qmi_err_num)
{

  *num_of_entries = GetGSBConfigFromXML(gsb_conf);

  if (!*num_of_entries )
  {
    LOG_MSG_ERROR("No configured entries found",0,0,0);
    *qmi_err_num = QMI_ERR_INTERNAL_V01;
    return false;
  }

  return true;
}

