/**********************************************************************
*  Copyright (c) 2013-2017 Qualcomm Technologies, Inc.
*  All Rights Reserved.
*  Confidential and Proprietary - Qualcomm Technologies, Inc.
**********************************************************************/

#ifndef __MODULE_IMGLIB_H__
#define __MODULE_IMGLIB_H__

#include "camera_dbg.h"
#include "modules.h"
#include "mct_stream.h"
#include "mgr/scene_detect_mgr.h"

#define MODULE_IMGLIB_PORT_NAME_LEN 32
#define MODULE_IMGLIB_MAX_NAME_LENGH 50
#define MODULE_IMGLIB_STATIC_PORTS 3

/* Imglib internal configuration limits */

/* Max number of modules in internal topology */
#define MODULE_IMGLIB_MAX_TOPO_MOD 10
/* Max parallel topologies per port. Should not exceed 31 */
#define MODULE_IMGLIB_MAX_PAR_TOPO 5
/* Max events which can be hold inside imglib waiting for ack
 * from internal topologies. Currently is used only for buffer divert event */
#define MODULE_IMGLIB_PORT_MAX_HOLD_EVENTS 20
/* Max params which can be stored */
#define MODULE_IMGLIB_MAX_STORED_PARAMS 4

/* Port events mask features flags */

/* Use internal buffers and do not block buff divert event.
 * NOTE: Number of buffers allocated are max hold events */
#define MODULE_IMGLIB_PORT_USE_INT_BUFS (1 << 0)
/* Parse super buffer param (sent as separate parameters) */
#define MODULE_IMGLIB_PORT_PARSE_SUPER_PARAMS (1 << 1)
/* Forward event from one to all other parallel topologies*/
#define MODULE_IMGLIB_PORT_PAR_TOPO_SHARING_EVENT (1 << 2)

/** module_imglib_session_params_t
 *   @mutex: Mutex protecting session params
 *   @params: Session params buffer
 *   @sessionid: Session Id
 *
 *   imglib session params holder structure
 **/
typedef struct {
  pthread_mutex_t mutex;
  parm_buffer_t params;
  unsigned int sessionid;
} module_imglib_session_params_t;

/** module_imglib_t
 *   @topo_attached: Parallel topologies attached in this holder
 *   @topo_list: Array of parallel topologies lists holding topology modules.
 *   @port_events_mask: Mask containing port events features
 *   @params_to_restore: Array of flags indicating which parameters
 *     need to be restored for this port.
 *
 *   Imglib topology holder structure. Holds topology information.
 **/
typedef struct {
  uint32_t topo_attached;
  uint32_t port_events_mask;
  boolean params_to_restore[CAM_INTF_PARM_MAX];
  mct_list_t *topo_list[MODULE_IMGLIB_MAX_PAR_TOPO];
} module_imglib_topo_holder_t;

/** module_imglib_t
 *   @dummy_port: Dummy port needed for module events
 *   @imglib_modules: List holding pointers to active image modules
 *   @topology: Pointer to array of topology holders
 *   @session_list_mutex: Session params list mutex
 *   @params_to_store: Array of flags indicating parameters
 *     need to be stored for all topologies
 *   @session_params_list: List containing session parameters
 *   @scene_detect_mgr: Scene detection manager
 *
 *   Imglib module structure
 **/
typedef struct {
  mct_port_t *dummy_port;
  mct_list_t *imglib_modules;
  module_imglib_topo_holder_t *topology;
  pthread_mutex_t session_list_mutex;
  boolean params_to_store[CAM_INTF_PARM_MAX];
  mct_list_t *session_params_list;
  img_scene_detect_mgr_t scene_detect_mgr;
} module_imglib_t;

/**
 * Function: module_imglib_create_port
 *
 * Description: Create imglib port
 *
 * Arguments:
 *   @p_mct_mod: Pointer to imglib module
 *   @dir: Port direction
 *   @static_p: static created port
 *   @num_mirror_ports: Number of mirror ports to be
 *     created. Mirror ports are used for conecting
 *     internal topologies.
 *
 * Return values:
 *   MCTL port pointer \ NULL on fail
 *
 * Notes: Currently supported only source ports
 **/
mct_port_t *module_imglib_create_port(mct_module_t *p_mct_mod,
  mct_port_direction_t dir, boolean static_p, uint32_t num_mirror_ports);

/**
 * Function: module_imglib_free_port
 *
 * Description: This function is used to free the imglib ports
 *
 * Arguments:
 *   @p_mct_mod: Mctmodule instance pointer
 *   @p_port: Mct port which need to be freed
 *
 * Return values:
 *   TRUE/FALSE
 *
 * Notes: none
 **/
boolean module_imglib_free_port(mct_module_t *p_mct_mod, mct_port_t *p_port);

/**
 * Function: module_imglib_create_dummy_port
 *
 * Description: Create imglib dummy port
 *
 * Arguments:
 *   p_mct_mod - Pointer to imglib module
 *   dir - Port direction
 *
 * Return values:
 *     MCTL port pointer \ NULL on fail
 *
 * Notes: Currently supported only source ports
 **/
mct_port_t *module_imglib_create_dummy_port(mct_module_t *p_mct_mod,
  mct_port_direction_t dir);

/**
 * Function: module_imglib_get_port_with_identity
 *
 * Description: Search for reserved port with given identity
 *
 * Arguments:
 *   @p_mct_mod: Pointer to imglib module
 *   @identity: Identity to search for
 *   @dir: Port direction
 *
 * Return values:
 *     MCTL port pointer \ NULL on fail
 *
 **/
mct_port_t *module_imglib_get_port_with_identity(mct_module_t *p_mct_mod,
  unsigned int identity, mct_port_direction_t dir);

/**
 * Function: module_imglib_get_dyn_port_with_sessionid
 *
 * Description: Search for reserved dynamic port with given session id
 *
 * Arguments:
 *   @p_mct_mod - Pointer to imglib module
 *   @sessionid - Session id to serch for
 *   @dir - Port direction
 *
 * Return values:
 *     MCTL port pointer \ NULL on fail
 *
 **/
mct_port_t *module_imglib_get_dyn_port_with_sessionid(mct_module_t *p_mct_mod,
  unsigned int sessionid, mct_port_direction_t dir);

/**
 * Function: module_imglib_get_and_reserve_port
 *
 * Description: Search and reserve available port
 *
 * Arguments:
 *   @p_mct_mod - Pointer to imglib module
 *   @stream_info - Stream info
 *   @dir - Port direction
 *   @peer_cap - peer port capabilites
 *
 * Return values:
 *     MCTL port pointer \ NULL on fail
 *
 **/
mct_port_t *module_imglib_get_and_reserve_port(mct_module_t *p_mct_mod,
   mct_stream_info_t *stream_info, mct_port_direction_t dir, void *peer_cap);

/**
 * Function: module_imglib_get_topology
 *
 * Description: Get internal topology based in stream info
 *
 * Arguments:
 *   @module: Imagelib module object
 *   @stream_info: mct_stream_info_t struct

 * Return values:
 *     Pointer to topology holder on success \ NULL on fail
 *
 * Notes: none
 **/
module_imglib_topo_holder_t *module_imglib_get_topology(mct_module_t *module,
  mct_stream_info_t *stream_info);

/**
 * Function: module_imglib_get_session_params
 *
 * Description: Function used to get session parameters based on session id
 *
 * Arguments:
 *   @module: Mct module instance
 *   @sessionid: Session id
 *
 * Return values:
 *   Pointer to imglib session parameter struct
 *
 * Notes: none
 **/
module_imglib_session_params_t *module_imglib_get_session_params(
  mct_module_t *module, unsigned int sessionid);

/**
 * Function: module_imglib_store_session_params
 *
 * Description: Function used to store session params
 *
 * Arguments:
 *   @module: mct module pointer
 *   @param_to_store: Params need to be stored
 *   @sessionid: Session id
 *
 * Return values:
 *   TRUE/FALSE
 *
 * Notes: none
 **/
boolean module_imglib_store_session_params(mct_module_t *module,
  mct_event_control_parm_t *param_to_store, unsigned int sessionid);

/**
 * Function: module_cac_set_parent
 *
 * Description: Interface function for set parent of cac module
 *
 * Arguments:
 *   @p_mct_mod: Cac module
 *   @p_parent: Stream object to be set as cac module parent

 * Return values:
 *     none
 *
 * Notes: This is only temporal
 **/
void module_cac_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/**
 * Function: module_afs_set_parent
 *
 * Description: Interface function for set parent of afs module
 *
 * Arguments:
 *   @p_mct_mod: afs module
 *   @p_parent: Stream object to be set as afs module parent

 * Return values:
 *     none
 *
 * Notes: This is only temporal
 **/
void module_afs_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/**
 * Function: module_wnr_set_parent
 *
 * Description: Interface function for set parent of wnr module
 *
 * Arguments:
 *   @p_mct_mod: WNR module
 *   @p_parent: Stream object to be set as wnr module parent

 * Return values:
 *     none
 *
 * Notes: This is only temporal
 **/
void module_wnr_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/** module_dis20_set_parent:
 *
 *  Arguments:
 *  @p_parent - parent module pointer
 *
 * Description: This function is used to set the parent pointer
 * of the dis 2.0 module
 *
 * Return values:
 *     none
 *
 * Notes: none
 **/
void module_dis20_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/** module_llvd_set_parent:
 *
 *  Arguments:
 *  @p_parent - parent module pointer
 *
 * Description: This function is used to set the parent pointer
 * of the LLVD module
 *
 * Return values:
 *     none
 *
 * Notes: none
 **/
void module_llvd_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/** module_lcac_set_parent:
 *
 *  Arguments:
 *  @p_parent - parent module pointer
 *
 * Description: This function is used to set the parent pointer
 * of the LCAC module
 *
 * Return values:
 *     none
 *
 * Notes: none
 **/
void module_lcac_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/** module_sw2d_set_parent:
 *
 *  Arguments:
 *   @p_mct_mod - MCT module instance pointer
 *   @p_parent - parent module pointer
 *
 * Description: This function is used to set the parent pointer
 *   of the SW2D module
 *
 * Return values:
 *   none
 *
 * Notes: none
 **/
void module_sw2d_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/** module_sw_tnr_set_parent:
 *
 *  Arguments:
 *  @p_parent - parent module pointer
 *
 * Description: This function is used to set the parent pointer
 * of the SW TNR module
 *
 * Return values:
 *     none
 *
 * Notes: none
 **/
void module_sw_tnr_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

/** module_afs_init:
 *
 *  Arguments:
 *  @name - name of the module
 *
 * Description: This function is used to initialize the afs module
 *
 * Return values:
 *     MCTL module instance pointer
 *
 * Notes: none
 **/

/** module_ppeiscore_set_parent:
 *
 *  Arguments:
 *  @p_parent - parent module pointer
 *
 * Description: This function is used to set the parent pointer
 * of the ppeiscore module
 *
 * Return values:
 *     none
 *
 * Notes: none
 **/
void module_ppeiscore_set_parent(mct_module_t *p_mct_mod, mct_module_t *p_parent);

mct_module_t *module_afs_init(const char *name);

/**
 * Function: module_afs_free_mod
 *
 * Description: This function is used to free the afs module
 *
 * Arguments:
 *   p_mct_mod - MCTL module instance pointer
 *
 * Return values:
 *     none
 *
 * Notes: none
 **/
void module_afs_deinit(mct_module_t *p_mct_mod);

/* Exported APIs */
mct_module_t *module_chroma_flash_init(const char *name);
void module_chroma_flash_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_optizoom_init(const char *name);
void module_optizoom_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_ubifocus_init(const char *name);
void module_ubifocus_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_refocus_init(const char *name);
void module_refocus_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_trueportrait_init(const char *name);
void module_trueportrait_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_llvd_init(const char *name);
void module_llvd_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_lcac_init(const char *name);
void module_lcac_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_sw_tnr_init(const char *name);
void module_sw_tnr_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_dis20_init(const char *name);
void module_dis20_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_stillmore_init(const char *name);
void module_stillmore_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_dcrf_init(const char *name);
void module_dcrf_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_sw2d_init(const char *name);
void module_sw2d_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_oem_feat1_init(const char *name);
void module_oem_feat1_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_sample_init(const char *name);
void module_imgbase_deinit(mct_module_t *p_mct_mod);
void module_imgbase_set_parent(mct_module_t *p_mct_mod,
  mct_module_t *p_parent);
mct_module_t *module_paaf_init(const char *name);
void module_paaf_deinit(mct_module_t *p_mct_mod);
void module_paaf_set_parent(mct_module_t *p_mct_mod,
  mct_module_t *p_parent);
mct_module_t *module_ppeiscore_init(const char *name);
void module_ppeiscore_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_quadracfa_init(const char *name);
mct_module_t *module_svhdr_init(const char *name);
void module_svhdr_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_sat_init(const char *name);
void module_sat_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_bincorr_init(const char *name);
void module_bincorr_deinit(mct_module_t *p_mct_mod);
mct_module_t *module_qdc_common_init(const char *name);
void module_qdc_common_deinit(mct_module_t *p_mct_mod);

#endif //__MODULE_IMGLIB_H__
