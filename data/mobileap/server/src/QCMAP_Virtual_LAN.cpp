/*====================================================

FILE:  QCMAP_virtual_LAN.cpp

SERVICES:
   QCMAP Virtual LAN Specific Implementation

=====================================================

  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.

=====================================================*/
/*======================================================
  EDIT HISTORY FOR MODULE

  Please notice that the changes are listed in reverse chronological order.

  when       who        what, where, why
  --------   ---        -------------------------------------------------------
  03/15/17   jc         Created

======================================================*/
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if !defined(FEATURE_DATA_TARGET_MDM9607) && !defined(FEATURE_MOBILEAP_APQ_PLATFORM) && !defined(FEATURE_QCMAP_OFFTARGET)
#include <linux/msm_ipa.h>
#include <sys/ioctl.h>
#include <net/if.h>
#endif

#include "ds_string.h"
#include "ds_util.h"
#include "QCMAP_Virtual_LAN.h"
#include "QCMAP_ConnectionManager.h"
#include "qcmap_cm_api.h"
#include "QCMAP_LAN.h"
#include "QCMAP_Tethering.h"
#include "QCMAP_L2TP.h"

bool QCMAP_Virtual_LAN::flag = false;
QCMAP_Virtual_LAN* QCMAP_Virtual_LAN::object=NULL;
qcmap_vlan_id_list_t QCMAP_Virtual_LAN::vlanIDList;
/*=====================================================
                             Class Definitions
  =====================================================*/

/*=====================================================
                             VLAN Helper functions
  =====================================================*/

/*==========================================================
 FUNCTION    qcmap_match_vlan_id
===========================================================*/
/*!
@brief
  Comparator function for match vlan id in vlan list

@parameters
  Two void pointers with vlan_id as first and vlan id as second.

@return
  0 - on a match
  1 - otherwise

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
long int QCMAP_Virtual_LAN::qcmap_match_vlan_id
(
  const void *first,
  const void *second
)
{
  uint16* vlan_id = NULL;
/*------------------------------------------------------------------------*/
  if( (first == NULL) || (second == NULL))
  {
    return 1;
  }
  vlan_id = ( uint16*)second;

  return((*vlan_id == *(uint16 *)first) ? false : true );
}
/*==========================================================
 FUNCTION    qcmap_match_vlan_id_in_list
===========================================================*/
/*!
@brief
  Comparator function for match vlan id in vlan list

@parameters
  Two void pointers with vlan list type as first and vlan id as second.

@return
  0 - on a match
  1 - otherwise

@note

@ Dependencies
   - None

@ Side Effects
   - None
*/
/*==========================================================*/
long int QCMAP_Virtual_LAN::qcmap_match_vlan_id_in_list
(
  const void *first,
  const void *second
)
{
  uint16* vlan_id = NULL;
/*------------------------------------------------------------------------*/
  if( (first == NULL) || (second == NULL))
  {
    return 1;
  }
  qcmap_vlan_list_item_t* vlan_node = (qcmap_vlan_list_item_t*)first;
  vlan_id = ( uint16*)second;

  return((*vlan_id == vlan_node->vlan_id) ? false : true );
}
/*=====================================================
                 VLAN Static Functions.
  =====================================================*/
/*=====================================================
  FUNCTION DeleteVLANConfigToXML
======================================================*/
/*!
@brief
  Deletes VLAN Config to XML.

@parameters
qcmap_msgr_vlan_config_v01 vconfig

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

boolean QCMAP_Virtual_LAN::DeleteVLANConfigFromXML
(
  qcmap_msgr_vlan_config_v01 vconfig
)
{
  pugi::xml_document xml_file;
  pugi::xml_node root, child;
  char data[MAX_STRING_LENGTH] = {0};
  QCMAP_ConnectionManager* QcMapMgr=
                        QCMAP_ConnectionManager::Get_Instance(NULL,false);
  char xml_iface[MAX_STRING_LENGTH] = {0};
  char xml_vlan_id[MAX_STRING_LENGTH] = {0};
  boolean found = false;
/*------------------------------------------------------------------------*/
  if(!xml_file.load_file(QcMapMgr->xml_path))
  {
    LOG_MSG_ERROR("Unable to load XML file.",0,0,0);
    return false;
  }
  /* Delete child with matching config*/
  root =
       xml_file.child(System_TAG).child(MobileAPCfg_TAG).child(VLAN_TAG);

  snprintf(data,MAX_STRING_LENGTH,"%d",vconfig.vlan_id);
  for (child = root.first_child(); child; child = child.next_sibling())
  {
     if (!strncmp(child.name(), VCONFIG_TAG,strlen(VCONFIG_TAG)))
     {
       strlcpy(xml_iface,child.child(VCONFIG_IFACE_TAG).child_value(),
                                                      MAX_STRING_LENGTH);
       strlcpy(xml_vlan_id,child.child(VCONFIG_ID_TAG).child_value(),
                                                     MAX_STRING_LENGTH);
       if (!strncmp(xml_iface,vconfig.local_iface,strlen(vconfig.local_iface))
           &&
           !strncmp(xml_vlan_id,data,strlen(data)))
       {
         LOG_MSG_ERROR("deleting vlan config with iface=%s, vlan_id %s from xml",
                       xml_iface,xml_vlan_id, 0);
         root.remove_child(child);
         QcMapMgr->WriteConfigToXML(UPDATE_MOBILEAP_XML, &xml_file);
         found = true;
         break;
       }
     }
  }
  return found;

}

/*=====================================================
  FUNCTION DeleteVLANConfig
======================================================*/
/*!
@brief
  Deletes VLAN Config and brings down VLAN.

@parameters
  qcmap_msgr_vlan_config_v01 vlan_config,
  void *softApHandle,
  qmi_error_type_v01 *qmi_err_num

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/
boolean QCMAP_Virtual_LAN::DeleteVLANConfig
(
  qcmap_msgr_vlan_config_v01 vlan_config,
  void *softApHandle,
  qmi_error_type_v01 *qmi_err_num
)
{
  QCMAP_Virtual_LAN_phy_iface_type phy_iface_type = QCMAP_TETH_MIN;
  ds_dll_el_t * search_node = NULL;
  qcmap_vlan_info_list_t* vlan_list;
  qcmap_vlan_list_item_t* vlan_config_node = NULL;
  *qmi_err_num= QMI_ERR_NONE_V01;
  QCMAP_Virtual_LAN* QcMapVLANMgr=
                                  QCMAP_Virtual_LAN::Get_Instance(false);
  QCMAP_L2TP* QcMapL2TPMgr= QCMAP_L2TP::Get_Instance(false);
  qcmap_vlan_id_list_t* vlan_id_list;
  ds_dll_el_t * node = NULL;
  /*------------------------------------------------------------------------*/
  /* VLAN manager cannot be NULL as VLAN object will be created at MobileAP
  enable. So returning, error if VLAN object is NULL*/
  if (!QcMapVLANMgr)
  {
    LOG_MSG_ERROR("DeleteVLANConfig(),VLAN object is not created yet",0,0,0);
    *qmi_err_num= QMI_ERR_INTERNAL_V01;
    return false;
  }
  phy_iface_type = QCMAP_Virtual_LAN::GetIfaceTypeFromIface(
                                                     vlan_config.local_iface);
  if (QCMAP_TETH_MIN == phy_iface_type)
  {
    LOG_MSG_ERROR("DeleteVLANConfig(),Unsupported iface-name:%s passed in config",
                  vlan_config.local_iface,0,0);
    *qmi_err_num= QMI_ERR_INVALID_ARG_V01;
    return false;
  }

  if (MIN_VLAN_ID > vlan_config.vlan_id || MAX_VLAN_ID < vlan_config.vlan_id)
  {
    LOG_MSG_ERROR("DeleteVLANConfig(),Unsupported VLAN-ID:%d passed in config",
                  vlan_config.vlan_id,0,0);
    *qmi_err_num= QMI_ERR_INVALID_ARG_V01;
    return false;
  }

  /*Delete vlan ID frm vlanID list*/
  vlan_id_list = &(QCMAP_Virtual_LAN::vlanIDList);
  search_node = ds_dll_delete (vlan_id_list->VlanIDHead,
                               &(vlan_id_list->VlanIDTail),
                               (void*)&(vlan_config.vlan_id),
                               qcmap_match_vlan_id);

  if (NULL == search_node)
  {
    *qmi_err_num= QMI_ERR_NO_EFFECT_V01;
    LOG_MSG_ERROR("DeleteVLANConfig(),VLAN ID %d doesnot exist",
                  vlan_config.vlan_id,0,0);
    return false;
  }

    /*Delete vlan config to vlan_list and update xml file*/
  vlan_list = &(QcMapVLANMgr->physical_iface[phy_iface_type].vlan_list);
  search_node = ds_dll_search (vlan_list->VLANListHead,
                               (void*)&(vlan_config.vlan_id),
                               qcmap_match_vlan_id_in_list);

  if (NULL == search_node)
  {
    *qmi_err_num= QMI_ERR_NO_EFFECT_V01;
    LOG_MSG_ERROR("DeleteVLANConfig(),VLAN ID %d list item not found",
                  vlan_config.vlan_id,0,0);
    return false;
  }

  vlan_config_node = (qcmap_vlan_list_item_t*)ds_dll_data(search_node);

  if (vlan_config_node == NULL)
  {
    LOG_MSG_ERROR("DeleteVLANConfig() - VLAN info NULL", 0,0,0);
    ds_dll_delete (vlan_list->VLANListHead,
                   &vlan_list->VLANListTail,
                   (void*)&(vlan_config.vlan_id),
                   qcmap_match_vlan_id_in_list);
    ds_dll_free(search_node);
    search_node = NULL;
    *qmi_err_num= QMI_ERR_INTERNAL_V01;
    return false;
  }

    /*If phy link is enabled, Delete VLAN*/
  if (QcMapVLANMgr->physical_iface[phy_iface_type].link_up)
  {
    /*Uinstall and clean L2TP's on top of this VLAN*/
    if (QcMapL2TPMgr)
    {
      if(!QcMapL2TPMgr->InstallDelTunnelsOnVLANIface(
                                    vlan_config_node->iface_name,false))
      {
        LOG_MSG_ERROR("DeleteVLANConfig(), Install/Delete L2TP tunnels on VLAN id %d not succeded",
                      vlan_config_node->vlan_id,0,0);
      }
    }
    else
    {
      LOG_MSG_INFO1("DeleteVLANConfig(),L2TP instance NULL",0,0,0);
    }

    QcMapVLANMgr->DeleteVLAN(*vlan_config_node);
    vlan_config_node->is_up = false;
  }

  search_node = ds_dll_delete (vlan_list->VLANListHead,
                               &(QcMapVLANMgr->physical_iface[phy_iface_type].\
                                                  vlan_list.VLANListTail),
                               (void*)&(vlan_config.vlan_id),
                               qcmap_match_vlan_id_in_list);


  ds_free(vlan_config_node);
  vlan_config_node = NULL;
  if (search_node)
  {
    ds_dll_free(search_node);
    search_node = NULL;
  }

  QCMAP_Virtual_LAN::DeleteVLANConfigFromXML(vlan_config);
  return true;
}

/*=====================================================
  FUNCTION GetPhyIfaceVLANIDFromIface
======================================================*/
/*!
@brief
  Gets physical iface type, vlan ID from iface name.

@parameters
  char *iface_name,\
  QCMAP_Virtual_LAN_phy_iface_type *phy_type,\
  uint16 *vlan_id @output pram

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

boolean QCMAP_Virtual_LAN::GetPhyIfaceVLANIDFromIface(char *iface_name,\
                                   QCMAP_Virtual_LAN_phy_iface_type *phy_type,\
                                   uint16 *vlan_id)
{
  QCMAP_Virtual_LAN_phy_iface_type phy_iface_type = QCMAP_TETH_MIN;
  char  phy_iface_name[QCMAP_MAX_IFACE_NAME_SIZE_V01] = {0};
  char  vlan_id_text[QCMAP_MAX_IFACE_NAME_SIZE_V01] = {0};
  char *ptr = NULL;
/*------------------------------------------------------------------------*/
  if (iface_name == NULL || phy_type == NULL || vlan_id == NULL)
  {
    LOG_MSG_ERROR("GetPhyIfaceVLANIDFromIface() - Invalid input params",\
                                                0,0,0);
    return false;
  }

  ptr = strchr(iface_name,'.');
  if (ptr == NULL)
  {
    strlcpy(phy_iface_name,iface_name,QCMAP_MAX_IFACE_NAME_SIZE_V01);
    *vlan_id = 0;
  }
  else
  {
    strlcpy(phy_iface_name,iface_name,(ptr-iface_name+1));
    strlcpy(vlan_id_text,ptr+1,QCMAP_MAX_IFACE_NAME_SIZE_V01);
    *vlan_id = atoi(vlan_id_text);
  }
  phy_iface_type = QCMAP_Virtual_LAN::GetIfaceTypeFromIface(phy_iface_name);

  if (phy_iface_type == QCMAP_TETH_MIN)
  {
    LOG_MSG_ERROR("GetPhyIfaceVLANIDFromIface() - Invalid physical interface:"
                  "%s in iface_name %s",phy_iface_name,iface_name,0);
    return false;
  }

  *phy_type = phy_iface_type;
  return true;
}

QCMAP_Virtual_LAN_phy_iface_type QCMAP_Virtual_LAN::GetIfaceTypeFromIface
(
  char *iface_name
)
{
  QCMAP_Virtual_LAN_phy_iface_type ret_val = QCMAP_TETH_MIN;
/*------------------------------------------------------------------------*/
   if (!strncmp(iface_name,ECM_IFACE ,strlen(ECM_IFACE)))
   {
      ret_val = QCMAP_TETH_ECM;
   }
   else if (!strncmp(iface_name, RNDIS_IFACE, strlen(RNDIS_IFACE)))
   {
      ret_val = QCMAP_TETH_RNDIS;
   }
   else if (!strncmp(iface_name, ETH_IFACE, strlen(ETH_IFACE)))
   {
      ret_val = QCMAP_TETH_ETH;
   }
   else if (!strncmp(iface_name, BRIDGE_IFACE, strlen(BRIDGE_IFACE)))
   {
      ret_val = QCMAP_TETH_BRIDGE;
   }

   LOG_MSG_INFO1("GetIfaceTypeFromIface() : ret_val %d",ret_val, 0, 0);
   return ret_val;
}

/*=====================================================
  FUNCTION GetVLANConfig
======================================================*/
/*!
@brief
  Gets VLAN Config.

@parameters
  qcmap_msgr_vlan_config_v01 *vlan_config,
  uint32 *length,
  qmi_error_type_v01 *qmi_err_num

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/
boolean QCMAP_Virtual_LAN::GetVLANConfig
(
  qcmap_msgr_vlan_config_v01 *vlan_config,
  uint32 *length,
  qmi_error_type_v01 *qmi_err_num
)
{
  QCMAP_Virtual_LAN* QcMapVLANMgr=
                                  QCMAP_Virtual_LAN::Get_Instance(false);
  ds_dll_el_t * node = NULL;
  qcmap_vlan_list_item_t* vlan_config_item;
  *qmi_err_num= QMI_ERR_NONE_V01;
  qcmap_vlan_info_list_t  *vlan_list;
  int num_vlan_config = 0;
  *qmi_err_num= QMI_ERR_NONE_V01;
  /*------------------------------------------------------------------------*/
  if (NULL == vlan_config || NULL == length)
  {
    LOG_MSG_ERROR("GetVLANConfig(),Invalid parameters passedt\n",0,0,0);
    *qmi_err_num= QMI_ERR_INVALID_ARG_V01;
    return false;
  }
  /* VLAN manager cannot be NULL as VLAN object will be created at MobileAP
  enable. So returning, error if VLAN object is NULL*/
  if (!QcMapVLANMgr)
  {
    LOG_MSG_ERROR("GetVLANConfig(),VLAN object is not created yet\n",0,0,0);
    *qmi_err_num= QMI_ERR_INTERNAL_V01;
    return false;
  }

  for (int i =0; i < QCMAP_MAX_PHY_LAN_IFACE; i++)
  {
    if (NULL != QcMapVLANMgr->physical_iface[i].vlan_list.VLANListHead)
    {
      vlan_list = &QcMapVLANMgr->physical_iface[i].vlan_list;
      node = vlan_list->VLANListHead;
      node = ds_dll_next (node, (const void**)(&vlan_config_item));
      while (node \
            && \
            node != vlan_list->VLANListHead)
      {
        strlcpy(vlan_config[num_vlan_config].local_iface,
                QcMapVLANMgr->physical_iface[i].iface_name,
                QCMAP_MAX_IFACE_NAME_SIZE_V01);
        vlan_config[num_vlan_config].vlan_id = vlan_config_item->vlan_id;

        num_vlan_config++;
        if (num_vlan_config == QCMAP_MSGR_MAX_VLAN_ENTRIES_V01)
        {
          LOG_MSG_ERROR("GetVLANConfig(),MAX VLAN entries reached,"\
                        "return err\n",0,0,0);
          *qmi_err_num = QMI_ERR_INTERNAL_V01;
          break;
        }
        node = ds_dll_next (node, (const void**)(&vlan_config_item));
      }
    }
  }
  *length = num_vlan_config;
  return (*qmi_err_num == QMI_ERR_NONE_V01 ? true : false);
}

/*===========================================================================
  FUNCTION GetVLANNodeforVLANID
==========================================================================*/
/*!
@brief
  Gets vlan node associated to vlan id.

@parameters
 uint16 vlan_id

@return
  qcmap_vlan_list_item_t *
  NULL if not found

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
qcmap_vlan_list_item_t* QCMAP_Virtual_LAN::GetVLANNodeforVLANID
(
  uint16 vlan_id
)
{
  qcmap_vlan_info_list_t* vlan_list = NULL;
  ds_dll_el_t * search_node = NULL;
  qcmap_vlan_list_item_t* vlan_config_node = NULL;
  QCMAP_Virtual_LAN* QcMapVLANMgr=
                                  QCMAP_Virtual_LAN::Get_Instance(false);
/*------------------------------------------------------------------------*/

  if (!QcMapVLANMgr)
  {
    LOG_MSG_ERROR("GetVLANNodeforVLANID() - NULL QcMapVLANMgr", 0,0,0);
    return NULL;
  }

  for (int i = 0; i < QCMAP_MAX_PHY_LAN_IFACE; i++)
  {

    vlan_list = &(QcMapVLANMgr->physical_iface[i].vlan_list);
    search_node = ds_dll_search (vlan_list->VLANListHead,
                               (void*)&(vlan_id),
                               qcmap_match_vlan_id_in_list);

    if (NULL == search_node)
    {
      continue;
    }

    vlan_config_node = (qcmap_vlan_list_item_t*)ds_dll_data(search_node);

    if (vlan_config_node == NULL)
    {
      LOG_MSG_ERROR("GetVLANNodeforVLANID() - VLAN info NULL", 0,0,0);
      ds_dll_delete (vlan_list->VLANListHead,
                   &vlan_list->VLANListTail,
                   (void*)&(vlan_id),
                   qcmap_match_vlan_id_in_list);
      ds_dll_free(search_node);
      search_node = NULL;
      return NULL;
    }
    if (vlan_config_node->is_up)
    {
      LOG_MSG_INFO1("GetVLANNodeforVLANID() returnng vlan node for vlan_id %d"
                     " corresponding iface is up",vlan_id,0,0);
      return vlan_config_node;
    }
    else
    {
      LOG_MSG_INFO1("GetVLANNodeforVLANID() returnng NULL for vlan_id %d"
                     " corresponding iface is not up",vlan_id,0,0);
      return NULL;
    }
  }

  LOG_MSG_ERROR("GetVLANNodeforVLANID() vlan id %d not found",
                vlan_id,0,0);
  return NULL;
}

/*===========================================================================
  FUNCTION GetIPAddrforVLAN
==========================================================================*/
/*!
@brief
  Returns IP addresses of VLAN iface corresponding to vlan id. passed

@parameters
  uint16 vlan_id,
  qcmap_ip4_addr_subnet_mask_v01 *ipv4_addr,
  qcmap_ip6_addr_prefix_len_data_v01 *ipv6_addr

@return
  true if vlan id is configured
  fasle if otherwise

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
boolean QCMAP_Virtual_LAN::GetIPAddrforVLAN
(
  uint16 vlan_id,
  qcmap_ip4_addr_subnet_mask_v01 *ipv4_addr,
  qcmap_ip6_addr_prefix_len_v01 *ipv6_addr
)
{
  qcmap_vlan_id_list_t* vlan_list = NULL;
  qcmap_vlan_list_item_t* vlan_config_node = NULL;
  ds_dll_el_t * search_node = NULL;
/*------------------------------------------------------------------------*/
  if (ipv4_addr == NULL || ipv6_addr == NULL || vlan_id == 0)
  {
    LOG_MSG_ERROR("GetIPAddrforVLAN() Invalid input parameters",
                    0,0,0);
    return false;
  }

  vlan_list = &(QCMAP_Virtual_LAN::vlanIDList);
  search_node = ds_dll_search (vlan_list->VlanIDHead,
                               (void*)&(vlan_id),
                               qcmap_match_vlan_id);

  if (NULL == search_node)
  {
    LOG_MSG_ERROR("GetIPAddrforVLAN(),VLAN id:%d is not configured\n",
                  vlan_id,0,0);
    return false;
  }

  vlan_config_node = QCMAP_Virtual_LAN::GetVLANNodeforVLANID(vlan_id);

  /*If vlan_config node is null, it means that vlan iface is not up.
  So return false
  */
  if (NULL == vlan_config_node)
  {
    LOG_MSG_ERROR("GetIPAddrforVLAN(),VLAN id:%d is not found or "
                  "not up\n",vlan_id,0,0);
    return false;
  }

  ipv4_addr->addr = vlan_config_node->ipv4_addr;
  ipv4_addr->subnet_mask = inet_addr(VLAN_SUBNET_MASK);

  for (int i =0; i<QCMAP_MSGR_IPV6_ADDR_LEN_V01;i++)
  {
    ipv6_addr->addr[i] = vlan_config_node->ipv6_addr.s6_addr[i];
  }
  ipv6_addr->prefix_len = VLAN_IPV6_PREFIX_LEN;

  LOG_MSG_INFO1("GetIPAddrforVLAN() Returning IP address for VLAN",
                    0,0,0);
  return true;
}
/*===========================================================================
  FUNCTION IsVLANIDUp
==========================================================================*/
/*!
@brief
  Returns if vlan id is up or not.

@parameters
 uint16 vlan_id
 char *iface_name - buffer to fill interface name

@return
  true if vlan id is configured
  fasle if otherwise

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
boolean QCMAP_Virtual_LAN::IsVLANIDUp( uint16 vlan_id, char *iface_name )
{
  qcmap_vlan_id_list_t* vlan_list = NULL;
  qcmap_vlan_list_item_t* vlan_config_node = NULL;
  ds_dll_el_t * search_node = NULL;
  QCMAP_Virtual_LAN* QCMAPVlan = QCMAP_Virtual_LAN::Get_Instance(false);
/*------------------------------------------------------------------------*/
  if (!QCMAPVlan)
  {
    LOG_MSG_ERROR("VLAN instance has not been created",0,0,0);
    return false;
  }

  if (vlan_id == 0)
  {
    LOG_MSG_ERROR("Invalid input parameters",0,0,0);
    return false;
  }

  vlan_list = &(QCMAP_Virtual_LAN::vlanIDList);
  search_node = ds_dll_search (vlan_list->VlanIDHead,
                               (void*)&(vlan_id),
                               qcmap_match_vlan_id);

  if (NULL == search_node)
  {
    LOG_MSG_ERROR("VLAN id:%d is not configured", vlan_id,0,0);
    return false;
  }

  vlan_config_node = QCMAP_Virtual_LAN::GetVLANNodeforVLANID(vlan_id);

  /*If vlan_config node is null means vlan ID is not configured or
  and internal error occured and could not retrivw vlan info.
  Return false in either case.
  */
  if (NULL == vlan_config_node)
  {
    LOG_MSG_ERROR("VLAN id:%d is not found or not up", vlan_id,0,0);
    return false;
  }

  //if iface name is not null fill it in
  if (iface_name != NULL)
  {
    LOG_MSG_INFO1("Returning iface name %s", iface_name,0,0);
    strlcpy(iface_name, vlan_config_node->iface_name, QCMAP_MSGR_INTF_LEN);
  }

  LOG_MSG_INFO1("VLAN id:%d is installed. returning true", vlan_id,0,0);
  return true;
}

/*=====================================================
  FUNCTION SetVLANConfig
======================================================*/
/*!
@brief
  Sets VLAN Config and brings up VLAN.

@parameters
  qcmap_msgr_vlan_config_v01 vlan_config,
  void *softApHandle,
  qmi_error_type_v01 *qmi_err_num

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/
boolean QCMAP_Virtual_LAN::SetVLANConfig
(
  qcmap_msgr_vlan_config_v01 vlan_config,
  void *softApHandle,
  qmi_error_type_v01 *qmi_err_num
)
{
  QCMAP_Virtual_LAN_phy_iface_type phy_iface_type = QCMAP_TETH_MIN;
  ds_dll_el_t * search_node = NULL;
  qcmap_vlan_list_item_t* node;
  qcmap_vlan_list_item_t new_node;
  qcmap_vlan_id_list_t* vlan_list;
  QCMAP_Virtual_LAN* QcMapVLANMgr=
                                  QCMAP_Virtual_LAN::Get_Instance(false);
  *qmi_err_num= QMI_ERR_NONE_V01;
  uint16     *vlan_id_node = NULL;
  QCMAP_L2TP* QcMapL2TPMgr= QCMAP_L2TP::Get_Instance(false);
/*------------------------------------------------------------------------*/
  phy_iface_type = QCMAP_Virtual_LAN::GetIfaceTypeFromIface(
                                                     vlan_config.local_iface);
  if (QCMAP_TETH_MIN == phy_iface_type)
  {
    LOG_MSG_ERROR("SetVLANConfig(),Unsupported iface-name:%s passed"\
                  " in config",vlan_config.local_iface,0,0);
    *qmi_err_num= QMI_ERR_INVALID_ARG_V01;
    return false;
  }

  if (MIN_VLAN_ID > vlan_config.vlan_id || MAX_VLAN_ID < vlan_config.vlan_id)
  {
    LOG_MSG_ERROR("SetVLANConfig(),Unsupported VLAN-ID:%d passed in config",
                                                vlan_config.vlan_id,0,0);
    *qmi_err_num= QMI_ERR_INVALID_ARG_V01;
    return false;
  }
  if (!QcMapVLANMgr)
  {
    LOG_MSG_ERROR("SetVLANConfig(),VLAN object is not created yet,"\
                  "Set config to xml",0,0,0);
    QCMAP_Virtual_LAN::SetVLANConfigToXML(vlan_config);
    return true;
  }

  /*Add vlan config to vlan_list and update xml file*/
  vlan_list = &(QCMAP_Virtual_LAN::vlanIDList);
  search_node = ds_dll_search (vlan_list->VlanIDHead,
                               (void*)&(vlan_config.vlan_id),
                               qcmap_match_vlan_id);

  if (NULL != search_node)
  {
    LOG_MSG_ERROR("SetVLANConfig(),VLAN id:%d is already configured",
                  vlan_config.vlan_id,0,0);
    *qmi_err_num= QMI_ERR_NO_EFFECT_V01;
    return false;
  }

  memset(&new_node,0,sizeof(qcmap_vlan_list_item_t));
  new_node.phy_iface_type = phy_iface_type;
  if(!(QcMapVLANMgr->ConstructVLANNode(vlan_config,&new_node)))
  {
    LOG_MSG_ERROR("SetVLANConfig(): Error in constructing VLAN List node",
                  0,0,0);
    *qmi_err_num= QMI_ERR_INTERNAL_V01;
    return false;
  }

  if(!(QcMapVLANMgr->AddVlanIDEntryToList(vlan_config.vlan_id, vlan_id_node)))
  {
    LOG_MSG_ERROR("SetVLANConfig(): Error in adding node to vlan ID list",
                  0,0,0);
    *qmi_err_num= QMI_ERR_INTERNAL_V01;
    return false;
  }
  new_node.is_up = QcMapVLANMgr->physical_iface[phy_iface_type].link_up;

  if(!(QcMapVLANMgr->AddVLANEntryToList(new_node, node, phy_iface_type)))
  {
    LOG_MSG_ERROR("SetVLANConfig(): Error in adding node to vlan list",
                  0,0,0);
    *qmi_err_num= QMI_ERR_INTERNAL_V01;
    return false;
  }

  /*If phy link is enabled, configure VLAN*/
  if (QcMapVLANMgr->physical_iface[phy_iface_type].link_up)
    QcMapVLANMgr->ConfigureVLAN(new_node);

  sleep(VLAN_SLEEP_INTERVAL);
  if(new_node.is_up &&
     QcMapL2TPMgr->InstallDelTunnelsOnVLANIface(new_node.iface_name,
                                                    true))
  {
    LOG_MSG_ERROR("SetVLANConfig(),Install/Delete L2TP tunnels"
                  "on VLAN id %d not succeded",new_node.vlan_id,0,0);
  }

  QCMAP_Virtual_LAN::SetVLANConfigToXML(vlan_config);

  LOG_MSG_ERROR("SetVLANConfig - configure vlan link up %d",
                    QcMapVLANMgr->physical_iface[phy_iface_type].link_up,0,0);

  return true;
}

/*=====================================================
  FUNCTION SetVLANConfigToXML
======================================================*/
/*!
@brief
  Set VLAN Config to XML.

@parameters
qcmap_msgr_vlan_config_v01 vconfig

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

boolean QCMAP_Virtual_LAN::SetVLANConfigToXML
(
  qcmap_msgr_vlan_config_v01 vconfig
)
{
  pugi::xml_document xml_file;
  pugi::xml_node root, child, subchild;
  char data[MAX_STRING_LENGTH] = {0};
  QCMAP_ConnectionManager* QcMapMgr=
                        QCMAP_ConnectionManager::Get_Instance(NULL,false);
/*------------------------------------------------------------------------*/
  if(!xml_file.load_file(QcMapMgr->xml_path))
  {
    LOG_MSG_ERROR("Unable to load XML file.",0,0,0);
    return false;
  }

  LOG_MSG_ERROR("SetVLANConfigToXML - Save VLAN config",
                    0,0,0);
  /* Append new vconfig to existing vlan tag*/
  root =
       xml_file.child(System_TAG).child(MobileAPCfg_TAG).child(VLAN_TAG);

  child = root.append_child(VCONFIG_TAG);

  snprintf(data,MAX_STRING_LENGTH,"%s",vconfig.local_iface);
  subchild = child.append_child(VCONFIG_IFACE_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(data);
  LOG_MSG_ERROR("SetVLANConfigToXML - Save VLAN config added first node",
                    0,0,0);

  snprintf(data,MAX_STRING_LENGTH,"%d",vconfig.vlan_id);
  subchild = child.append_child(VCONFIG_ID_TAG);
  subchild.append_child(pugi::node_pcdata).set_value(data);

  LOG_MSG_ERROR("SetVLANConfigToXML - Save VLAN config added nodes",
                    0,0,0);
  QcMapMgr->WriteConfigToXML(UPDATE_MOBILEAP_XML,&xml_file);

  LOG_MSG_ERROR("SetVLANConfigToXML - Exit Save VLAN config",
                    0,0,0);
  return true;

}

/*=====================================================
                 Class Cunstuctor, Destructors.
  =====================================================*/

/*=====================================================
  FUNCTION Get_Instance
======================================================*/
/*!
@brief
  Gets and return instance of class QCMAP_Virtual_LAN

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

QCMAP_Virtual_LAN* QCMAP_Virtual_LAN::Get_Instance(boolean obj_create)
{
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  if(!flag && obj_create)
  {
    LOG_MSG_INFO1("Creating object : Virtual LAN",0, 0, 0);
    object = new QCMAP_Virtual_LAN();
    flag = true;
    return object;
  }
  else
  {
    return object;
  }
}
/*=====================================================
  FUNCTION Constructor
======================================================*/
/*!
@brief
  Initializes Virtual LAN variables.

@parameters
none

@return
  true  - on success
  false - on failure

@note
- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

QCMAP_Virtual_LAN::QCMAP_Virtual_LAN()
{
  char v6add_str[INET6_ADDRSTRLEN] = {0};
  QCMAP_LAN* QCMAPLANMgr=QCMAP_LAN::Get_Instance(false);

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  memset(this->physical_iface,0,\
         QCMAP_MAX_PHY_LAN_IFACE*sizeof(phy_iface_type_t));

  snprintf(v6add_str,sizeof(v6add_str),"%s:%d::%s",VLAN_BASE_IPV6_ADDR,
                                                   0,
                                                   STATIC_IID);
  for (int i = 0; i < QCMAP_MAX_PHY_LAN_IFACE; i++)
  {
    switch (i)
    {
      case QCMAP_TETH_ECM:
        strlcpy(this->physical_iface[i].iface_name, ECM_IFACE,
                                          QCMAP_MAX_IFACE_NAME_SIZE_V01);
        inet_pton(AF_INET,TETH_IFACE_LL_ADDR1,
                  &this->physical_iface[i].ipv4_addr);
      break;
      case QCMAP_TETH_RNDIS:
        strlcpy(this->physical_iface[i].iface_name, RNDIS_IFACE,
                                        QCMAP_MAX_IFACE_NAME_SIZE_V01);
        inet_pton(AF_INET,TETH_IFACE_LL_ADDR1,
                  &this->physical_iface[i].ipv4_addr);
      break;
      case QCMAP_TETH_ETH:
        strlcpy(this->physical_iface[i].iface_name, ETH_IFACE,
                                       QCMAP_MAX_IFACE_NAME_SIZE_V01);
        inet_pton(AF_INET,TETH_IFACE_LL_ADDR2,
                  &this->physical_iface[i].ipv4_addr);
      break;
      case QCMAP_TETH_BRIDGE:
        strlcpy(this->physical_iface[i].iface_name, BRIDGE_IFACE,
                                         QCMAP_MAX_IFACE_NAME_SIZE_V01);
        inet_pton(AF_INET,APPS_LAN_IP_ADDR,
                  &this->physical_iface[i].ipv4_addr);
      break;
      default:
        LOG_MSG_ERROR("QCMAP_Virtual_LAN() Not a valid iface_type",0,0,0);
        return;
      break;
   }
    inet_pton(AF_INET6,v6add_str,&this->physical_iface[i].ipv6_addr);
    memset(&this->physical_iface[i].vlan_list,0,sizeof(qcmap_vlan_info_list_t));
  }
  if (QCMAPLANMgr && QCMAPLANMgr->bridge_inited)
  {
    this->physical_iface[QCMAP_TETH_BRIDGE].link_up = true;
  }
  LOG_MSG_INFO1("Read VLAN config from XML",0, 0, 0);
  this->ReadVLANConfigFromXML();

}

/*======================================================
  FUNCTION Destructor
======================================================*/
/*!
@brief
  Destroyes the Virtual LAN Object.

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


QCMAP_Virtual_LAN::~QCMAP_Virtual_LAN()
{
  qcmap_vlan_info_list_t* vlan_list = NULL;
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  ds_dll_delete_all(QCMAP_Virtual_LAN::vlanIDList.VlanIDHead);

  for (int i = 0; i < QCMAP_MAX_PHY_LAN_IFACE; i++)
  {
    vlan_list = &this->physical_iface[i].vlan_list;
    ds_dll_delete_all(vlan_list->VLANListHead);
    vlan_list = NULL;
  }

  flag=false;
  object=NULL;
}

/*=====================================================
                 VLAN Public Functions.
  =====================================================*/
/*===========================================================================
  FUNCTION AddDeleteVLANOnIface
==========================================================================*/
/*!
@brief
  Configures VLAN on the iface passed.

@parameters
 char * iface_name

@return
- None

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
void QCMAP_Virtual_LAN::AddDeleteVLANOnIface
(
  char* iface_name,
  boolean link_up,
  uint32 ipv4_addr
)
{
  QCMAP_Virtual_LAN_phy_iface_type phy_iface_type = QCMAP_TETH_MIN;
  QCMAP_L2TP* QcMapL2TPMgr= QCMAP_L2TP::Get_Instance(false);
  ds_dll_el_t * node = NULL;
  qcmap_vlan_list_item_t *vlan_config = NULL;
  ds_dll_el_t *head_node = NULL;
/*------------------------------------------------------------------------*/
  phy_iface_type = QCMAP_Virtual_LAN::GetIfaceTypeFromIface(iface_name);
  if (QCMAP_TETH_MIN == phy_iface_type)
  {
    LOG_MSG_ERROR("AddDeleteVLANOnTethIface(),Unsupported iface name passed"\
                  " in config\n",0,0,0);
    return;
  }

  /*Save linkup flag and configure/delete vlan's on that interface*/
  if(VALIDATE_PHY_TYPE(phy_iface_type))
  {
    LOG_MSG_ERROR("phy_iface_type Not a valid QCMAP_Virtual_LAN_phy_iface_type,phy_iface_type value: %d ",phy_iface_type,0,0);
    return;
  }
  this->physical_iface[phy_iface_type].link_up = link_up;
  this->physical_iface[phy_iface_type].ipv4_addr = ipv4_addr;

  if (NULL != this->physical_iface[phy_iface_type].vlan_list.VLANListHead)
  {
    node = this->physical_iface[phy_iface_type].vlan_list.VLANListHead;
    node = ds_dll_next (node, (const void**)(&vlan_config));
    while (node &&
           node != this->physical_iface[phy_iface_type].vlan_list.VLANListHead)
    {
      if (link_up)
      {
        vlan_config->is_up = true;
        this->ConfigureVLAN(*vlan_config);
      }
      else
      {
        this->DeleteVLAN(*vlan_config);
        vlan_config->is_up = false;
      }

      /*Install L2TP on this VLAN*/
      if (QcMapL2TPMgr)
      {
         /*Sleep is required here to give enough time for route setup to complete*/
        sleep(VLAN_SLEEP_INTERVAL);
        if(!QcMapL2TPMgr->InstallDelTunnelsOnVLANIface(vlan_config->iface_name,
                                                      link_up))
        {
          LOG_MSG_ERROR("AddDeleteVLANOnTethIface(),Install/Delete L2TP tunnels"
                      "on VLAN id %d not succeded",vlan_config->vlan_id,0,0);
        }
      }
      else
      {
        LOG_MSG_INFO1("AddDeleteVLANOnTethIface(),L2TP instance NULL",0,0,0);
      }

      node = ds_dll_next (node, (const void**)(&vlan_config));
    }
  }

  return;
}

/*===========================================================================
  FUNCTION AddVLANEntryToList
==========================================================================*/
/*!
@brief
  Adds VLAN entry to List

@parameters
  qcmap_vlan_list_item_t vlan_config_node,
  qcmap_vlan_list_item_t* vlan_node
  QCMAP_Virtual_LAN_phy_iface_type iface_type

@return
  fasle- on failure
  true - on success

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
boolean QCMAP_Virtual_LAN::AddVLANEntryToList
(
  qcmap_vlan_list_item_t vlan_config_node,
  qcmap_vlan_list_item_t* vlan_node,
  QCMAP_Virtual_LAN_phy_iface_type iface_type
)
{

  LOG_MSG_INFO1("Entering AddVLANEntryToList",0,0,0);
  ds_dll_el_t * node = NULL;

  if(VALIDATE_PHY_TYPE(iface_type))
  {
    LOG_MSG_ERROR("iface_type Not a valid QCMAP_Virtual_LAN_phy_iface_type, iface_type value: %d",iface_type,0,0);
    return false;
  }

  qcmap_vlan_info_list_t* vlanList =
                         &(this->physical_iface[iface_type].vlan_list);

  if (vlanList && (vlanList->VLANListHead == NULL) )
  {
    /*The first node which is created is a dummy node which does not store any device
          information. This is done to make use of the doubly linked list framework which
           is already existing*/
    if (( node = ds_dll_init(NULL)) == NULL)
    {
      LOG_MSG_ERROR("AddVLANEntryToList - Error in allocating memory for node",
                    0,0,0);
      return false;
    }
    vlanList->VLANListHead = node;
  }

  vlan_node = (qcmap_vlan_list_item_t*)ds_malloc(
                                    sizeof(qcmap_vlan_list_item_t));
  if( vlan_node == NULL )
  {
    LOG_MSG_ERROR("AddVLANEntryToList - Error in allocating memory for"
                 "vlan entry",0,0,0);
    return false;
  }


  memset(vlan_node, 0, sizeof(qcmap_vlan_list_item_t));

  memcpy(vlan_node,&vlan_config_node,sizeof(qcmap_vlan_list_item_t));

  LOG_MSG_INFO1("AddVLANEntryToList - After copying vlan iface %s, vlan id %d",
                vlan_node->iface_name,vlan_node->vlan_id,0);
  //Store the VLAN entry in the linked list
  if (vlanList && (node = ds_dll_enq(vlanList->VLANListHead,
                         NULL, (void*)vlan_node)) == NULL)
  {
    LOG_MSG_ERROR("AddVLANEntryToList - Error in adding a node",0,0,0);
    ds_free(vlan_node);
    return false;
  }
  if(vlanList)
    vlanList->VLANListTail = node;

  LOG_MSG_INFO1("AddVLANEntryToList - Returning from enqueue",0,0,0);
  return true;

}
/*===========================================================================
  FUNCTION AddVlanIDEntryToList
==========================================================================*/
/*!
@brief
  Adds VLAN ID entry to list

@parameters
  uint16 vlan_id,
  uint16 *vlan_id_node

@return
  fasle- on failure
  true - on success

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
boolean QCMAP_Virtual_LAN::AddVlanIDEntryToList
(
  uint16 vlan_id,
  uint16 *vlan_id_node
)
{

  LOG_MSG_INFO1("Entering AddVlanIDEntryToList",0,0,0);
  ds_dll_el_t * node = NULL, *temp_node = NULL;

  qcmap_vlan_id_list_t* vlanList =
                         &(QCMAP_Virtual_LAN::vlanIDList);

  if (vlanList->VlanIDHead == NULL )
  {
    /*The first node which is created is a dummy node which does not store any device
          information. This is done to make use of the doubly linked list framework which
           is already existing*/
    if (( node = ds_dll_init(NULL)) == NULL)
    {
      LOG_MSG_ERROR("AddVlanIDEntryToList - Error in allocating memory"
                    " for node",0,0,0);
      return false;
    }
    vlanList->VlanIDHead = node;
  }

  vlan_id_node = (uint16*)ds_malloc(
                                    sizeof(uint16));
  if( vlan_id_node == NULL )
  {
    LOG_MSG_ERROR("AddVlanIDEntryToList - Error in allocating memory for"
                 "VLAN ID entry",0,0,0);
    return false;
  }


  memset(vlan_id_node, 0, sizeof(uint16));

  *vlan_id_node = vlan_id;

  if ((node = ds_dll_enq(vlanList->VlanIDHead,
                         NULL, (void*)vlan_id_node)) == NULL)
  {
    LOG_MSG_ERROR("AddVlanIDEntryToList - Error in adding a node",0,0,0);
    ds_free(vlan_id_node);
    return false;
  }
  vlanList->VlanIDTail = node;

  return true;
}
/*===========================================================================
  FUNCTION ConfigureVLAN
==========================================================================*/
/*!
@brief
  Configures VLAN.

@parameters
 qcmap_vlan_list_item_t vlan_config_node

@return
  None

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
void QCMAP_Virtual_LAN::ConfigureVLAN
(
  qcmap_vlan_list_item_t vlan_config_node
)
{
  char command[MAX_COMMAND_STR_LEN] = {0};
  char ipv6_addr[INET6_ADDRSTRLEN] = {0};
  struct in6_addr ipv6_prefix;
  char ipv4_addr[INET_ADDRSTRLEN] = {0};
  QCMAP_Backhaul_WWAN *QcMapBackhaulWWAN = NULL;
  QCMAP_Backhaul *QcMapBackhaul;
/*------------------------------------------------------------------------*/
  memset(&ipv6_prefix,0,sizeof(ipv6_prefix));
  if (VALIDATE_PHY_TYPE(vlan_config_node.phy_iface_type))
  {
    LOG_MSG_ERROR("ConfigureVLAN(),Not a valid QCMAP_Virtual_LAN_phy_iface_type"
                   ",iface_type value: %d",vlan_config_node.phy_iface_type,0,0);
    return;
  }

  snprintf( command, MAX_COMMAND_STR_LEN, "vconfig add %s %d",
               this->physical_iface[vlan_config_node.phy_iface_type].iface_name,
               vlan_config_node.vlan_id);
  ds_system_call(command,strlen(command));

  /* Bringup VLAN interface with V6 and V4 addresses*/
  if(!inet_ntop(AF_INET,(void *)&vlan_config_node.ipv4_addr,ipv4_addr,
                     INET_ADDRSTRLEN))
  {
    LOG_MSG_ERROR("ConfigureVLAN(),Error converting IPv4 address",
                  0,0,0);
    return;
  }

#if !defined(FEATURE_DATA_TARGET_MDM9607) && !defined(FEATURE_MOBILEAP_APQ_PLATFORM) && !defined(FEATURE_QCMAP_OFFTARGET)

  /* Send IOCTL to Neutrino if the underlying interface is ETH.
  If it succeds, Send IOCTL to IPA with vlan information*/
  if (vlan_config_node.phy_iface_type == QCMAP_TETH_ETH )
  {
    if (this->UpdateNeutrinoWithVlanIoctl(vlan_config_node.vlan_id,true))
    {
      if(!this->UpdateIPAWithVlanIOCTL(vlan_config_node.iface_name,
                             vlan_config_node.vlan_id,true))
      {
        LOG_MSG_ERROR("ConfigureVLAN(),Failed VLAN IOCTL to IPA",0,0,0);
      }
    }
    else
    {
      LOG_MSG_ERROR("ConfigureVLAN(),Failed VLAN IOCTL to Neutrino",0,0,0);
    }
  }
#endif

  snprintf( command, MAX_COMMAND_STR_LEN, "ifconfig %s %s netmask %s up",
               vlan_config_node.iface_name, ipv4_addr, VLAN_SUBNET_MASK);
  ds_system_call(command,strlen(command));

  inet_ntop(AF_INET6, &vlan_config_node.ipv6_addr.s6_addr, ipv6_addr,
                               INET6_ADDRSTRLEN);
  snprintf(command, MAX_COMMAND_STR_LEN,
            "ip -6 addr add %s dev %s",ipv6_addr, vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

  /* Add prefix based route on the vlan interface*/
  memcpy(&ipv6_prefix,&vlan_config_node.ipv6_addr,sizeof(struct in6_addr));
  for (int i = 4; i < 8; i++)
  {
    ipv6_prefix.s6_addr16[i] = 0;
  }

  inet_ntop(AF_INET6, &ipv6_prefix.s6_addr, ipv6_addr,
                               INET6_ADDRSTRLEN);
  snprintf(command, MAX_COMMAND_STR_LEN,
            "ip -6 route add %s/%d dev %s",ipv6_addr,
                                           VLAN_IPV6_PREFIX_LEN,
                                           vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

  /* Loop through Backhaul objects till you get the one corresponding to correct vlan.
     If one exits... */
  DECLARE_HASH_MAP_ITERATOR_FOR_BACKHAUL;
  START_OF_HASH_MAP_FOR_BACKHAUL(QCMAP_ConnectionManager::QCMAP_Backhaul_Hash);
  while (END_OF_HASH_MAP_FOR_BACKHAUL(QCMAP_ConnectionManager::QCMAP_Backhaul_Hash))
  {
    QcMapBackhaulWWAN = GET_BACKHAUL_WWAN_OBJ;
    QcMapBackhaul = GET_BACKHAUL_OBJ_AND_INC;
    if (QcMapBackhaul->vlan_id == vlan_config_node.vlan_id)
    {
      LOG_MSG_INFO1("Found Backhaul tied to VLAN ID %d", vlan_config_node.vlan_id,0,0);
      break;
    }
  }

  //Secondary PDN
  if (QcMapBackhaulWWAN && !QcMapBackhaulWWAN->EnableVlanPdnRules())
  {
    LOG_MSG_ERROR("Unable to install rules",0,0,0);
  }
}

/*===========================================================================
  FUNCTION ConstructVLANNode
==========================================================================*/
/*!
@brief
  Constructs VLAN info based on iface and vlan_id

@parameters
  qcmap_msgr_vlan_config_v01 vconfig,
  qcmap_vlan_list_item_t *vlan_node

@return
   false - on failure
   true - on success

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/

boolean QCMAP_Virtual_LAN::ConstructVLANNode(qcmap_msgr_vlan_config_v01 vconfig,
                                             qcmap_vlan_list_item_t *vlan_node)
{
  char iface_name[QCMAP_MAX_IFACE_NAME_SIZE_V01] = {0};
  char v6add_str[INET6_ADDRSTRLEN] = {0};
  char v4addr_str[INET_ADDRSTRLEN] = {0};
  struct in6_addr ipv6_addr;
  uint32 ipv4_addr;
  uint16 sub_addr = 0;
/*------------------------------------------------------------------------*/
  if (NULL == vlan_node)
  {
    LOG_MSG_ERROR("ConstructVLANNode() Invalid VLAN Node passed",0,0,0);
    return false;
  }

  snprintf(iface_name,sizeof(iface_name),"%s.%d",vconfig.local_iface,
                                                     vconfig.vlan_id);
  strlcpy(vlan_node->iface_name, iface_name,
                                  strlen(iface_name)+1);

  vlan_node->vlan_id = vconfig.vlan_id;

  snprintf(v6add_str,sizeof(v6add_str),"%s:%d::%s",VLAN_BASE_IPV6_ADDR,
                                                       vconfig.vlan_id,
                                                          STATIC_IID);
  snprintf(v4addr_str,sizeof(v4addr_str),"%s",VLAN_BASE_IPV4_ADDR);

  inet_pton(AF_INET6,v6add_str,(void *)&ipv6_addr.s6_addr);
  memcpy(&vlan_node->ipv6_addr,&ipv6_addr,sizeof(ipv6_addr));

  /* left shift vlan ID 4 bits tand or with base address to form a unique subnet
  for this VLAN. OR the result with 1 to form unique IP address*/
  ipv4_addr = ntohl(inet_addr(v4addr_str));
  sub_addr = vlan_node->vlan_id;
  ipv4_addr = ipv4_addr | (sub_addr << 4);
  ipv4_addr = ipv4_addr | 1;
  ipv4_addr = htonl(ipv4_addr);
  memcpy(&vlan_node->ipv4_addr,&ipv4_addr,sizeof(ipv4_addr));

  return true;
}

/*===========================================================================
  FUNCTION DeleteVLAN
==========================================================================*/
/*!
@brief
  Deletes specific VLAN config.

@parameters
 qcmap_vlan_list_item_t vlan_config_node

@return
  None

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
void QCMAP_Virtual_LAN::DeleteVLAN
(
  qcmap_vlan_list_item_t vlan_config_node
)
{
  char command[MAX_COMMAND_STR_LEN] = {0};
  struct in6_addr ipv6_prefix;
  char ipv6_addr[INET6_ADDRSTRLEN] = {0};
  QCMAP_Backhaul_WWAN *QcMapBackhaulWWAN = NULL;
  QCMAP_Backhaul *QcMapBackhaul;
/*------------------------------------------------------------------------*/

  /* Loop through Backhaul objects till you get the one corresponding to correct vlan.
     If one exits... */
  DECLARE_HASH_MAP_ITERATOR_FOR_BACKHAUL;
  START_OF_HASH_MAP_FOR_BACKHAUL(QCMAP_ConnectionManager::QCMAP_Backhaul_Hash);
  while (END_OF_HASH_MAP_FOR_BACKHAUL(QCMAP_ConnectionManager::QCMAP_Backhaul_Hash))
  {
    QcMapBackhaulWWAN = GET_BACKHAUL_WWAN_OBJ;
    QcMapBackhaul = GET_BACKHAUL_OBJ_AND_INC;
    if (QcMapBackhaul->vlan_id == vlan_config_node.vlan_id)
    {
      LOG_MSG_INFO1("Found Backhaul tied to VLAN ID %d", vlan_config_node.vlan_id,0,0);
      break;
    }
  }

  //Secondary PDN
  if (QcMapBackhaulWWAN && !QcMapBackhaulWWAN->DisableVlanPdnRules())
  {
    LOG_MSG_ERROR("Unable to disable rules",0,0,0);
  }

  memset(&ipv6_prefix,0,sizeof(ipv6_prefix));
  /* Delete prefix based route on the vlan interface*/
  memcpy(&ipv6_prefix,&vlan_config_node.ipv6_addr,sizeof(struct in6_addr));
  for (int i = 4; i < 8; i++)
  {
    ipv6_prefix.s6_addr16[i] = 0;
  }

  inet_ntop(AF_INET6, &ipv6_prefix.s6_addr, ipv6_addr,
                               INET6_ADDRSTRLEN);
  snprintf(command, MAX_COMMAND_STR_LEN,
            "ip -6 route del %s/%d dev %s",ipv6_addr,
                                           VLAN_IPV6_PREFIX_LEN,
                                           vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

  snprintf(command, MAX_COMMAND_STR_LEN,
            "ip -4 addr flush dev %s",vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

  snprintf(command, MAX_COMMAND_STR_LEN,
            "ip -6 addr flush dev %s",vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

  snprintf(command, MAX_COMMAND_STR_LEN,
            "ifconfig %s down",vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

  snprintf(command, MAX_COMMAND_STR_LEN,
            "vconfig rem %s",vlan_config_node.iface_name);
  ds_system_call(command, strlen(command));

#if !defined(FEATURE_DATA_TARGET_MDM9607) && !defined(FEATURE_MOBILEAP_APQ_PLATFORM) && !defined(FEATURE_QCMAP_OFFTARGET)
  /* Send IOCTL to Neutrino if the underlying interface is ETH.
  If it succeds, Send IOCTL to IPA with vlan information*/
  if (vlan_config_node.phy_iface_type == QCMAP_TETH_ETH )
  {
    if (this->UpdateNeutrinoWithVlanIoctl(vlan_config_node.vlan_id,false))
    {
      if(!this->UpdateIPAWithVlanIOCTL(vlan_config_node.iface_name,
                             vlan_config_node.vlan_id,false))
        LOG_MSG_ERROR("ConfigureVLAN(),Failed VLAN IOCTL to IPA",0,0,0);
    }
    else
      LOG_MSG_ERROR("ConfigureVLAN(),Failed VLAN IOCTL to Neutrino",0,0,0);
  }

#endif

}


/*===========================================================================
  FUNCTION GetIPAddrForphyLink
==========================================================================*/
/*!
@brief
  Returns IP addresses of physical interface.

@parameters
  QCMAP_Virtual_LAN_phy_iface_type iface_type,
  uint32 *ipv4_addr,
  struct in6_addr *ipv6_addr

@return
  true if phy interface is up.
  fasle if otherwise

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
boolean QCMAP_Virtual_LAN::GetIPAddrOfphyLink
(
  QCMAP_Virtual_LAN_phy_iface_type iface_type,
  uint32 *ipv4_addr,
  struct in6_addr *ipv6_addr
)
{
  qcmap_vlan_id_list_t* vlan_list = NULL;
  qcmap_vlan_list_item_t* vlan_config_node = NULL;
  ds_dll_el_t * search_node = NULL;
/*------------------------------------------------------------------------*/
  if (ipv4_addr == NULL || ipv6_addr == NULL || iface_type <= QCMAP_TETH_MIN
      || iface_type >= QCMAP_MAX_PHY_LAN_IFACE)
  {
    LOG_MSG_ERROR("GetIPAddrForphyLink() Invalid input parameters",
                    0,0,0);
    return false;
  }

  if (!this->physical_iface[iface_type].link_up)
  {
    LOG_MSG_ERROR("GetIPAddrForphyLink() Physical link not up, return false",
                    0,0,0);
    return false;
  }

  ipv4_addr = this->physical_iface[iface_type].ipv4_addr;

  for (int i =0; i<QCMAP_MSGR_IPV6_ADDR_LEN_V01;i++)
  {
    ipv6_addr->s6_addr[i] =
                        this->physical_iface[iface_type].ipv6_addr.s6_addr[i];
  }

  LOG_MSG_INFO1("GetIPAddrforVLAN() Returning IP address for Phy interface",
                    0,0,0);
  return true;
}
/*===========================================================================
  FUNCTION IsPhyLinkUP
==========================================================================*/
/*!
@brief
  Returns if the physical interface is up.

@parameters
 QCMAP_Virtual_LAN_phy_iface_type iface_type

@return
  None

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
boolean QCMAP_Virtual_LAN::IsPhyLinkUP(
                              QCMAP_Virtual_LAN_phy_iface_type iface_type)
{
/*------------------------------------------------------------------------*/
   if (!(iface_type > QCMAP_TETH_MIN
         &&
         iface_type < QCMAP_MAX_PHY_LAN_IFACE))
   {
     LOG_MSG_ERROR("IsPhyLinkUP() Invalid iface type passed",
                    0,0,0);
     return false;
   }
   return this->physical_iface[iface_type].link_up;
}
/*=====================================================
  FUNCTION ReadVLANConfigFromXML
======================================================*/
/*!
@brief
  Reads VLAN Config from XML.

@parameters

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

boolean QCMAP_Virtual_LAN::ReadVLANConfigFromXML()
{
  pugi::xml_document xml_file;
  pugi::xml_node root, child;
  ds_dll_el_t *search_node;
  in_addr addr;
  char data[MAX_STRING_LENGTH] = {0};
  qcmap_msgr_vlan_config_v01 vlan_config_info;
  uint16_t vlan_id = 0;
  qcmap_vlan_list_item_t vlan_config_node;
  qcmap_vlan_list_item_t *vlan_node;
  QCMAP_Virtual_LAN_phy_iface_type phy_iface_type = QCMAP_TETH_MIN;
  uint16*   vlan_id_node = NULL;
  qcmap_vlan_id_list_t* vlan_list = NULL;
/*------------------------------------------------------------------------*/
  memset(&vlan_config_node,0,sizeof(qcmap_vlan_list_item_t));

  if(!xml_file.load_file(QCMAP_ConnectionManager::xml_path))
  {
    LOG_MSG_ERROR("Unable to load XML file.",0,0,0);
    return false;
  }

  root =
       xml_file.child(System_TAG).child(MobileAPCfg_TAG).child(VLAN_TAG);

  for (child = root.first_child(); child; child = child.next_sibling())
  {
    if (strncmp(child.name(),VCONFIG_TAG,strlen(VCONFIG_TAG)))
    {
      LOG_MSG_ERROR("ReadVLANConfigFromXML(),Invalid tag found in vlan config",
                                             0,0,0);
      return false;
    }
    memset(&vlan_config_info,0,sizeof(vlan_config_info));
    strlcpy(vlan_config_info.local_iface,
            child.child(VCONFIG_IFACE_TAG).child_value(),\
          QCMAP_MAX_IFACE_NAME_SIZE_V01);
    vlan_id = atoi(child.child(VCONFIG_ID_TAG).child_value());
    memcpy(&vlan_config_info.vlan_id,&vlan_id,sizeof(vlan_id));

    phy_iface_type = QCMAP_Virtual_LAN::GetIfaceTypeFromIface(
                                                vlan_config_info.local_iface);

    LOG_MSG_INFO1("Save Vlan config with phy iface %d, vlan_id %d",phy_iface_type, vlan_id, 0);
    if (QCMAP_TETH_MIN == phy_iface_type)
    {
      LOG_MSG_ERROR("ReadVLANConfigFromXML(),Invalid iface_name: %s",
                                             vlan_config_info.local_iface,0,0);
      return false;
    }

    vlan_config_node.phy_iface_type = phy_iface_type;
    vlan_list = &(QCMAP_Virtual_LAN::vlanIDList);

    search_node = ds_dll_search (vlan_list->VlanIDHead,
                               (void*)&(vlan_config_info.vlan_id),
                               qcmap_match_vlan_id);

    if (NULL != search_node)
    {
      LOG_MSG_ERROR("ReadVLANConfigFromXML(),Duplicate VLAN id:%d found",
                  vlan_config_info.vlan_id,0,0);
      return false;
    }

    if(!(this->ConstructVLANNode(vlan_config_info,\
                                  &vlan_config_node)))
    {
      LOG_MSG_ERROR("ReadVLANConfigFromXML() Error in constructing node",0,0,0);
      return false;
    }

    if(!(this->AddVlanIDEntryToList(vlan_id, vlan_id_node)))
    {
      LOG_MSG_ERROR("ReadVLANConfigFromXML(): Error in adding node to vlan "\
                    "ID list",0,0,0);
      return false;
    }

    if(this->AddVLANEntryToList(vlan_config_node,vlan_node,phy_iface_type)\
                                                                      == false)
    {
      LOG_MSG_ERROR("ReadVLANConfigFromXML() Error in adding vlan entry to list"
                    ,0,0,0);
      return false;
    }
  }
  return true;
}
#if !defined(FEATURE_DATA_TARGET_MDM9607) && !defined(FEATURE_MOBILEAP_APQ_PLATFORM) && !defined(FEATURE_QCMAP_OFFTARGET)
/*=====================================================
  FUNCTION UpdateIPAWithVlanIOCTL
======================================================*/
/*!
@brief
  Sends IOCTL to IPA with VLAN info.

@parameters

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
- None

- Side Effects
- None
*/
/*====================================================*/

boolean QCMAP_Virtual_LAN::UpdateIPAWithVlanIOCTL(char *iface_name,
                                              uint16 vlan_id,
                                              boolean is_up)
{
  int ioctl_file_fd = -1, ioctl_ret = -1;
  struct ipa_ioc_vlan_iface_info vlan_ioctl_buffer;
/*------------------------------------------------------------------------*/
  if (iface_name == NULL || vlan_id == 0)
  {
    LOG_MSG_ERROR("UpdateIPAWithIOCTL() Invalid parameters received",0,0,0);
    return false;
  }

  memset(&vlan_ioctl_buffer,0,sizeof(ipa_ioc_vlan_iface_info));
  ioctl_file_fd = open(IPA_DEVICE_NAME, O_RDWR);
  if (ioctl_file_fd < 0)
  {
    LOG_MSG_ERROR("UpdateIPAWithIOCTL() Cannot open file for ioctl",0,0,0);
    return false;
  }

  strlcpy(vlan_ioctl_buffer.name,iface_name,IPA_RESOURCE_NAME_MAX);
  vlan_ioctl_buffer.vlan_id = vlan_id;

  if (is_up)
  {
    ioctl_ret = ioctl(
                  ioctl_file_fd,IPA_IOC_ADD_VLAN_IFACE,&vlan_ioctl_buffer);
  }
  else
  {
    ioctl_ret = ioctl(
                  ioctl_file_fd,IPA_IOC_DEL_VLAN_IFACE,&vlan_ioctl_buffer);
  }

  if (ioctl_ret)
  {
    LOG_MSG_ERROR("UpdateIPAWithIOCTL() IOCTL to IPA failed with err %d, errno %s",
                  ioctl_ret,strerror(errno),0);
    close(ioctl_file_fd);
    return false;
  }

  close(ioctl_file_fd);
  return true;
}
/*=====================================================
  FUNCTION UpdateNeutrinoWithVlanIoctl
======================================================*/
/*!
@brief
  Sends IOCTL to Neutrino with VLAN info.

@parameters
  uint16 vlan_id,
  boolean is_up

@return
  true  - on Success
  false - on Failure

@note

- Dependencies
    This IOCTL packs forllowing data and sends over. If
    there is a change in data structure(struct ifr_data_struct_ipa)
    used by Neutrino, this needs to be updated.

- Side Effects
- None
*/
/*====================================================*/

boolean QCMAP_Virtual_LAN::UpdateNeutrinoWithVlanIoctl
(
  uint16 vlan_id,boolean is_up
)
{
  int ioctl_fd = -1,ioctl_ret = -1;
  unsigned char ioctl_buffer[NEUTRINO_IOCTL_BUFFER_SIZE_BYTES] = {0};
  struct   ifreq ifr;
  uint32 dma_c2 = NTN_TX_DMA_CH_2;
  uint32 dma_c0 = NTN_RX_DMA_CH_0;
  uint32 vlan_ipa_enable = NEUTRINO_IPA_VLAN_DISABLE;
  int    retry = 0;
/*------------------------------------------------------------------------*/

  if (vlan_id == 0)
  {
    LOG_MSG_ERROR("UpdateNeutrinoWithIoctl() Invalid parameters received",
                   0,0,0);
    return false;
  }

  ioctl_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (ioctl_fd < 0)
  {
    LOG_MSG_ERROR("UpdateNeutrinoWithIoctl() Cannot open file for ioctl to"
                  "neutrnio",0,0,0);
    return false;
  }

  memset(&ifr, 0, sizeof(struct ifreq));
  strlcpy((char *)ifr.ifr_name,this->physical_iface[QCMAP_TETH_ETH].iface_name,
                                IFNAMSIZ);

  memcpy(&ioctl_buffer[NEUTRION_IOCTL_TX_IPA_DMA_INDEX],&dma_c2,sizeof(dma_c2));
  memcpy(&ioctl_buffer[NEUTRION_IOCTL_RX_IPA_DMA_INDEX],&dma_c0,sizeof(dma_c0));
  vlan_ipa_enable = is_up ? NEUTRINO_IPA_VLAN_ENABLE:NEUTRINO_IPA_VLAN_DISABLE;

  memcpy(&ioctl_buffer[NEUTRION_IOCTL_COMMAND_INDEX],&vlan_ipa_enable,
                                                       sizeof(vlan_ipa_enable));
  memcpy(&ioctl_buffer[NEUTRION_IOCTL_VLAN_ID_INDEX],&vlan_id,sizeof(vlan_id));

  ifr.ifr_ifru.ifru_data = (void *)&ioctl_buffer;

  /* Retry IOCTL if the return value is DWC_ETH_QOS_CONFIG_FAIL (-3) for
     predefine MAX retries*/
  do
  {
    ioctl_ret = ioctl(ioctl_fd,SIOCDEVPRIVATE+1, &ifr);
    retry++;

    if (ioctl_ret && errno == VLAN_NTN_CONFIG_FAIL_ERRNO)
    {
      LOG_MSG_INFO1("UpdateNeutrinoWithIoctl() failed with errno %d, sleep",
                       VLAN_NTN_IOCTL_SLEEP_INTERVAL,0,0);
      sleep(VLAN_NTN_IOCTL_SLEEP_INTERVAL);
    }
    else
    {
      LOG_MSG_INFO1("UpdateNeutrinoWithIoctl() IOCTL ret val %d, errno %d",
                     ioctl_ret,errno,0);
      break;
    }

  }while(retry <= VLAN_NTN_MAX_RETRIES);


  if (ioctl_ret)
  {
    LOG_MSG_ERROR("UpdateNeutrinoWithIoctl() IOCTL to Neutrino failed",0,0,0);
    close(ioctl_fd);
    return false;
  }

  close(ioctl_fd);
  return true;

}
#endif
