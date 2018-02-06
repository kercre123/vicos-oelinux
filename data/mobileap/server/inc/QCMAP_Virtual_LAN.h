#ifndef _QCMAP_Virtual_LAN_H_
#define _QCMAP_Virtual_LAN_H_

/*======================================================

FILE:  QCMAP_Virtual_LAN.h

SERVICES:
   QCMAP Virtual LAN Class

=======================================================

  Copyright (c) 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.

======================================================*/
/*======================================================
  EDIT HISTORY FOR MODULE

  Please notice that the changes are listed in reverse chronological order.
    when       who        what, where, why
  --------   ---        -------------------------------------------------------
  03/15/17   jc           Created
======================================================*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>

#include "qualcomm_mobile_access_point_msgr_v01.h"
#include "ds_util.h"
#include "ds_list.h"
#include "qcmap_cm_api.h"
#include "ds_string.h"
#include "QCMAP_ConnectionManager.h"

/* Vconfig flags */
#define VLAN_TAG                     "vlan"
#define VCONFIG_TAG                  "VConfig"
#define VCONFIG_IFACE_TAG            "interface"
#define VCONFIG_ID_TAG               "id"

/*Static IPv6 prefixes*/
#define VLAN_BASE_IPV6_ADDR          "FD53:7CB8:0383"
#define STATIC_IID                   "0123"
#define VLAN_IPV6_PREFIX_LEN         64

/*Static IPv4 addresses*/
#define VLAN_BASE_IPV4_ADDR          "192.169.0.0"
#define VLAN_SUBNET_MASK             "255.255.255.240"

#define MIN_VLAN_ID                  1  /*vlan 0 is reserved as per RFC*/
#define MAX_VLAN_ID                  4094/*vlan 4095 is max and it is reserved*/

#define VLAN_SLEEP_INTERVAL          3
#define VLAN_NTN_IOCTL_SLEEP_INTERVAL 3
#define VLAN_NTN_MAX_RETRIES         2
#define VLAN_NTN_CONFIG_FAIL_ERRNO     11 /*EAGAIN 11*/

#if !defined(FEATURE_DATA_TARGET_MDM9607) && !defined(FEATURE_MOBILEAP_APQ_PLATFORM) && !defined(FEATURE_QCMAP_OFFTARGET)
  #define IPA_DEVICE_NAME                  "/dev/ipa"
  #define NTN_TX_DMA_CH_2                   2
  #define NTN_RX_DMA_CH_0                   0
  #define NEUTRINO_IOCTL_BUFFER_SIZE_BYTES  14
  #define NEUTRINO_IPA_VLAN_DISABLE         0
  #define NEUTRINO_IPA_VLAN_ENABLE          1

  #define NEUTRION_IOCTL_TX_IPA_DMA_INDEX   0
  #define NEUTRION_IOCTL_RX_IPA_DMA_INDEX   4
  #define NEUTRION_IOCTL_COMMAND_INDEX      8
  #define NEUTRION_IOCTL_VLAN_ID_INDEX      12
#endif

typedef enum {
  QCMAP_TETH_MIN = -1,
  QCMAP_TETH_ECM = 0,
  QCMAP_TETH_RNDIS = 1,
  QCMAP_TETH_ETH = 2,
  QCMAP_TETH_BRIDGE = 3,
  QCMAP_MAX_PHY_LAN_IFACE
}QCMAP_Virtual_LAN_phy_iface_type;

typedef struct
{
  ds_dll_el_t *VLANListHead;
  ds_dll_el_t *VLANListTail;
}qcmap_vlan_info_list_t;

typedef struct
{
  ds_dll_el_t *VlanIDHead;
  ds_dll_el_t *VlanIDTail;
}qcmap_vlan_id_list_t;

typedef struct
{
  uint16                      vlan_id;
  char                        iface_name[QCMAP_MAX_IFACE_NAME_SIZE_V01];
  struct in6_addr             ipv6_addr;
  uint32                      ipv4_addr;
  boolean                     is_up;
  QCMAP_Virtual_LAN_phy_iface_type phy_iface_type;
} qcmap_vlan_list_item_t;

typedef struct
{
  char                        iface_name[QCMAP_MAX_IFACE_NAME_SIZE_V01];
  boolean                     link_up;
  struct in6_addr             ipv6_addr;
  uint32                      ipv4_addr;
  qcmap_vlan_info_list_t      vlan_list;
} phy_iface_type_t;

class QCMAP_Virtual_LAN
{
private:

  static bool flag;
  static QCMAP_Virtual_LAN *object;
  phy_iface_type_t  physical_iface[QCMAP_MAX_PHY_LAN_IFACE];
  static qcmap_vlan_id_list_t vlanIDList;
  QCMAP_Virtual_LAN();

public:

  ~QCMAP_Virtual_LAN();
  static QCMAP_Virtual_LAN *Get_Instance(boolean obj_create=false);
  /* ----------------------Virtual LAN Execution---------------------------*/
  boolean ReadVLANConfigFromXML();

  boolean AddVLANEntryToList( qcmap_vlan_list_item_t vlan_config_info,
                             qcmap_vlan_list_item_t *vlan_config,
                             QCMAP_Virtual_LAN_phy_iface_type iface_type);

  boolean ConstructVLANNode(qcmap_msgr_vlan_config_v01 vconfig,
                            qcmap_vlan_list_item_t *vlan_node);

  void ConfigureVLAN(qcmap_vlan_list_item_t vlan_config_node);

  void DeleteVLAN(qcmap_vlan_list_item_t vlan_config_node);

  void AddDeleteVLANOnIface(char *iface_name, boolean link_up,
                                uint32 ipv4_addr);

  boolean IsPhyLinkUP(QCMAP_Virtual_LAN_phy_iface_type iface_type);

  boolean GetIPAddrOfPhyLink(QCMAP_Virtual_LAN_phy_iface_type iface_type,
                              uint32 *ipv4_addr,
                              struct in6_addr *ipv6_addr);

  boolean AddVlanIDEntryToList(uint16 vlan_id,
                                uint16 *vlan_id_node);

  boolean GetIPAddrOfphyLink(QCMAP_Virtual_LAN_phy_iface_type iface_type,
                             uint32 *ipv4_addr,
                             struct in6_addr *ipv6_addr);

#if !defined(FEATURE_DATA_TARGET_MDM9607) && !defined(FEATURE_MOBILEAP_APQ_PLATFORM) && !defined(FEATURE_QCMAP_OFFTARGET)
  boolean UpdateIPAWithVlanIOCTL(char *iface_name, uint16 vlan_id,boolean is_up);
  boolean UpdateNeutrinoWithVlanIoctl(uint16 vlan_id,boolean is_up);
#endif

  static boolean GetPhyIfaceVLANIDFromIface(char *iface_name,
                                   QCMAP_Virtual_LAN_phy_iface_type *phy_type,
                                   uint16 *vlan_id);

  static  boolean SetVLANConfigToXML(qcmap_msgr_vlan_config_v01 vconfig);

  static  boolean DeleteVLANConfigFromXML(qcmap_msgr_vlan_config_v01 vconfig);

  static long int qcmap_match_vlan_id_in_list( const void *first,
                                                         const void *second );

  static long int qcmap_match_vlan_id( const void *first,
                                                         const void *second );

  static boolean SetVLANConfig( qcmap_msgr_vlan_config_v01 vlan_config,
                                      void *softApHandle,
                                      qmi_error_type_v01 *qmi_err_num );

  static boolean GetVLANConfig( qcmap_msgr_vlan_config_v01 *vlan_config,
                                      uint32 *length,
                                      qmi_error_type_v01 *qmi_err_num );

  static boolean DeleteVLANConfig( qcmap_msgr_vlan_config_v01 vlan_config,
                                      void *softApHandle,
                                      qmi_error_type_v01 *qmi_err_num );

  static qcmap_vlan_list_item_t* GetVLANNodeforVLANID(uint16 vlan_id);

  static boolean IsVLANIDUp(uint16 vlan_id, char *iface_name);

  static boolean GetIPAddrforVLAN(uint16 vlan_id,
                                 qcmap_ip4_addr_subnet_mask_v01 *ipv4_addr,
                                 qcmap_ip6_addr_prefix_len_v01 *ipv6_addr);

  static QCMAP_Virtual_LAN_phy_iface_type GetIfaceTypeFromIface(
                                                     char *iface_name);

};
#endif
