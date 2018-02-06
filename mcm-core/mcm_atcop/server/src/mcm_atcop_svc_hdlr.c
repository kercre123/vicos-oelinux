/*===========================================================================

                         M C M _ ATCO P _ S V C _ H D L R. C

DESCRIPTION

  The MCM ATCOP Service Handler Source File.

EXTERNALIZED FUNCTIONS

  mcm_atcop_svc_init()
    Register the MCM ATCOP service with QMUX for all applicable QMI links.

 Copyright (c) 2013-2014 Qualcomm Technologies, Inc.  All Rights Reserved.
 Qualcomm Technologies Proprietary and Confidential.

===========================================================================*/
/*===========================================================================

when        who    what, where, why
--------    ---    ----------------------------------------------------------
06/05/14    pm     Removed dependency on dss_new. Defined log message macros
09/03/13    vm     Created module
===========================================================================*/

/*===========================================================================

                          INCLUDE FILES FOR MODULE

===========================================================================*/
#include <stdio.h>
#include "comdef.h"
#include "mcm_atcop_svc_hdlr.h"
#include "mcm_service_object_v01.h"
#include "mcm_common_v01.h"
#include "mcm_client_v01.h"
#include "mcm_ipc.h"

#include "mcm_atcop_v01.h"
#include "diag_lsm.h"
#include "ds_util.h"
#include "ds_list.h"
#include "comdef.h"

#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#ifdef USE_GLIB
#include <glib.h>
#define strlcpy g_strlcpy
#define strlcat g_strlcat
#define strlcmp g_strcmp0
#endif

#define MAX_BUF_LEN 2048
#define MCM_ATCOP_SVC_HANDLE 0x65432
#define MCM_ATCOP_DEV_FILE "/dev/smd8"

#define MCM_OK_STRING_LEN 4
#define MCM_ERROR_STRING_LEN 7
#define MCM_CME_STRING_LEN 12
#define MCM_CONNECT_STRING_LEN 9
#define MCM_ATCOP_COPN_CMD_LEN 5

#define MCM_ATCOP_AT_CMD "AT"
#define MCM_ATCOP_AT_CMD_LEN 2
#define MCM_ATCOP_ECHO_DISABLE "ATE0\r\n"
#define MCM_ATCOP_ECHO_DISABLE_LEN 6
#define MCM_ATCOP_CMD_DELIMITER ";"
#define MCM_ATCOP_ESC_CHARS "\r\n"
#define MCM_ATCOP_RESP_OK "OK\r\n"
#define MCM_ATCOP_RESP_ERROR1 "ERROR\r\n"
#define MCM_ATCOP_RESP_CME_ERROR "\r\n+CME ERROR"
#define MCM_ATCOP_RESP_CMS_ERROR "\r\n+CMS ERROR"
#define MCM_ATCOP_RESP_ERROR2 "\r\nERROR"
#define MCM_ATCOP_RESP_CONNECT1 "\r\nCONNECT"
#define MCM_ATCOP_RESP_CONNECT2  "CONNECT\r\n"
#define MCM_ATCOP_RESP_NO_CARRIER1 "\r\nNO CARRIER"
#define MCM_ATCOP_RESP_NO_CARRIER2 "NO CARRIER\r\n"

/* Set the MAX timeout for read to 5 mins */
#define MCM_ATCOP_MAX_READ_TIMEOUT 300


#define mcm_atcop_svc_LOG(...)                         \
  fprintf( stderr, "%s %d:", __FILE__, __LINE__); \
  fprintf( stderr, __VA_ARGS__ )

mcm_ipc_srv_mgr_exit_func atcop_exit;

typedef struct {
  qmi_client_handle clnt;
  unsigned int      client_fd;

}mcm_atcop_svc_client_info_type;

static boolean mcm_atcop_svc_inited = FALSE;


static mcm_atcop_svc_state_info_type mcm_atcop_svc_state;
static qmi_csi_os_params os_params;
static unsigned int first_client;
static int dev_fd; /* FD to Read and Write to /dev/smd8  */

#define UNSUPPORTED_CMD_COUNT 42
char *unsupported_cmds[UNSUPPORTED_CMD_COUNT] =
       {"ATD", "ATS", "ATO", "ATE", "ATQ",  "ATX", "ATV", "AT&E",
        "AT+IPR", "ATD" "ATS", "ATO", "ATE", "ATQ", "ATX", "ATV",
        "AT&E", "AT+IPR", "AT+ICF", "AT+IFC", "AT+ILRR", "ATT",
        "ATP", "ATA", "ATH", "ATL", "ATM", "AT+F", "AT+CHUP",
        "AT+CMGW", "AT$QCMGW", "ATL", "ATM", "AT+F", "AT+CHUP",
        "AT+CMGW", "AT$QCMGW", "ATL", "ATM", "AT+F", "AT+CHUP",
        "AT+CMGW", "AT$QCMGW", "ATL", "ATM", "AT+F", "AT+CHUP",
        "AT+CMGW", "AT$QCMGW", "AT$QCMGW", "AT$QCMGS"};

/*===========================================================================
LOG Msg Macros
=============================================================================*/
#define LOG_MSG_INFO1_LEVEL           MSG_LEGACY_MED
#define LOG_MSG_INFO2_LEVEL           MSG_LEGACY_MED
#define LOG_MSG_INFO3_LEVEL           MSG_LEGACY_LOW
#define LOG_MSG_ERROR_LEVEL           MSG_LEGACY_ERROR
#define PRINT_MSG( level, fmtString, x, y, z)                         \
        MSG_SPRINTF_4( MSG_SSID_LINUX_DATA, level, "%s(): " fmtString,      \
                       __FUNCTION__, x, y, z);
#define LOG_MSG_INFO1( fmtString, x, y, z)                            \
{                                                                     \
  PRINT_MSG( LOG_MSG_INFO1_LEVEL, fmtString, x, y, z);                \
}
#define LOG_MSG_INFO2( fmtString, x, y, z)                            \
{                                                                     \
  PRINT_MSG( LOG_MSG_INFO2_LEVEL, fmtString, x, y, z);                \
}
#define LOG_MSG_INFO3( fmtString, x, y, z)                            \
{                                                                     \
  PRINT_MSG( LOG_MSG_INFO3_LEVEL, fmtString, x, y, z);                \
}
#define LOG_MSG_ERROR( fmtString, x, y, z)                            \
{                                                                     \
  PRINT_MSG( LOG_MSG_ERROR_LEVEL, fmtString, x, y, z);                \
}

/*===========================================================================

                       FORWARD FUNCTION DECLARATIONS

===========================================================================*/

int mcm_atcop_svc_init
(
  void
);

static qmi_csi_cb_error mcm_atcop_svc_hdlr_client_connect_cb
(
  qmi_client_handle         client_handle,
  void                      *service_handle,
  void                      **connection_handle
);

static void mcm_atcop_svc_hdlr_client_disconnect_cb
(
  void                      *connection_handle,
  void                      *service_handle
);

static qmi_csi_cb_error mcm_atcop_svc_hdlr_client_req_cb
(
  void                     *connection_handle,
  qmi_req_handle           req_handle,
  unsigned int             msg_id,
  void                     *req_c_struct,
  unsigned int             req_c_struct_len,
  void                     *service_handle
);

static qmi_csi_cb_error  mcm_atcop_handle_req
(
  mcm_atcop_svc_client_info_type *clnt_info,
  qmi_req_handle                 req_handle,
  int                            msg_id,
  void                           *req_c_struct,
  int                            req_c_struct_len,
  void                           *service_handle
);

static qmi_csi_cb_error (* const req_handle_table[])
(
 mcm_atcop_svc_client_info_type *clnt_info,
 qmi_req_handle                 req_handle,
 int                            msg_id,
 void                           *req_c_struct,
 int                            req_c_struct_len,
 void                           *service_handle
) =
{
  mcm_atcop_handle_req,  /* Request handler for message ID 0x600 */
};

void mcm_atcop_exit_func();

void sighandler(int signal)
{
  mcm_error_t_v01 mcm_err_num = MCM_SUCCESS_V01;
  switch (signal)
  {
    case SIGTERM:
    case SIGINT:
      /* Close the Opened file on SIGINT*/
      if( dev_fd > 0 )
      {
        close(dev_fd);
        dev_fd = -1;
      }
      qmi_csi_unregister(mcm_atcop_svc_state.service_handle);
      exit(0);
      break;
    default:
      printf("Received unexpected signal %s\n", signal);
      break;
  }
}

/* Convert a string from lower case to upper. */
void str_to_upcase ( char *p )
{
  while( *p )
  {
    *p = toupper( *p );
    p++;
  }
}

/*****************************************************************************
  Below are the steps to be followed for validation:

  1. Each command should have a minimum of 2 characters
  2. Each command is separated by ;
  3. From the second command onwards AT is not mandatory.
  4. Below are the list of un supported commands.

 {"ATD"},{"ATS"},{"ATO"},{"ATE"},{"ATQ"},{"ATX"},{"ATV"},{"AT&C"},{"AT&D"},
 {"AT+IPR"},{"AT+ICF"},{"AT+IFC"},{"AT+ILRR"},{"ATT"},{"ATP"},{"ATA"},
 {"ATH"},{"ATL"},{"ATM"},{"AT+F"},{"AT+CHUP"},{"AT+CMGW"},{"AT$QCMGW"},
 {"AT$QCMGS"}, {"+CSMS"}, {"+CPMS"}, {"AT+CMGF"}, {"AT+CSCA"}, {"AT+CSMP"},
 {"AT+CSDH"}, {"AT+CSCB"}, {"AT+CNMI"}, {"AT+CMGL"}, {"AT+CMGR"}, {"AT+CNMA"},
 {"AT+CMGS"}, {"AT+CMSS"}, {"AT+CMGW"}, {"AT+CMGD"}, {"AT+CMGC"}, {"AT+CMMS"},
 {"AT+CSAS"}, {"AT+CRES"}

 *****************************************************************************/

int atcop_cmd_validate(char *command)
{
  unsigned short ch;
  int i;
  char *temp = command;
  int len = 0;

  if (command == NULL)
  {
    LOG_MSG_ERROR(" Invalid AT-COP Command ", 0, 0, 0);
    return MCM_ERROR_BADPARM_V01;
  }

  len = strlen(command);
  /* The first character should be A/a, Multiple A/a's can be present but should be
     followed by t/T */

  if ((command[0] == 'A') || (command[0] == 'a'))
  {
    i = 1;
    while(i <len)
    {
      if ((command[i] == 'A') || (command[i] == 'a'))
       i++;
      else if ((command[i] == 'T') || (command[i] == 't'))
        break;
      else
      {
        LOG_MSG_ERROR(" AT-COP Command not valid ", 0, 0, 0);
        return MCM_ERROR_BADPARM_V01;
      }
    }
  }
  else
  {
    LOG_MSG_ERROR(" AT-COP Command not valid ", 0, 0, 0);
    return MCM_ERROR_BADPARM_V01;
  }
  for(i= 0; i < UNSUPPORTED_CMD_COUNT; i ++)
  {
    if ( ! strncasecmp(command, unsupported_cmds[i], strlen(unsupported_cmds[i])) )
    {
      LOG_MSG_ERROR(" AT-COP Command not supported ", 0, 0, 0);
      return MCM_ERROR_GENERIC_V01;
    }
  }
  LOG_MSG_INFO1(" AT-COP Command validated successfully. ", 0, 0, 0);
  return 0;
}

/* Look for the last instance of target string. */
char *findlast(char *response, char *target_str)
{
  char *current;
  char *found = NULL;
  int target_length = 0;

  /* Length of target_str can be of any length. Hence
     length check not done here. */
  if ((response == NULL) || (target_str == NULL) ||
     (strlen(response) >= MCM_ATCOP_MAX_RESP_MSG_SIZE_V01))
  {
    LOG_MSG_ERROR(" Invalid arguments ", 0, 0, 0);
    return found;
  }

  target_length = strlen(target_str);
  current = response + strlen(response) - target_length;

  while ( current >= response )
  {
    if ((found = strstr(current, target_str)))
    {
      break;
    }
    current -= target_length;
  }
  return found;
}

/*===========================================================================
  FUNCTION IsDUNSoftAPMode
==========================================================================*/
/*!
@brief
  Checks if we are in DUN+SoftAP Mode or not.

@parameters
  None.

@return
  true  - If we are in DUN+SoftAP mode.
  flase - if we are not in DUN+SoftAP mode.

@note
- Dependencies
- None

- Side Effects
- None
*/
/*=========================================================================*/
static boolean IsDunSoftAPMode()
{
  int i = 0;
  FILE *cmd;
  char mode = 'N';
  const char process[] = "cat /data/usb/softap_w_dun";

  cmd = popen(process, "r");
  if( cmd)
  {
    mode = fgetc(cmd);
  }
  if ( mode == 'Y' )
  {
    LOG_MSG_INFO1("In DUN+SoftAP mode.", 0, 0, 0);
    return TRUE;
  }
  else
  {
    LOG_MSG_INFO1(" Not in DUN+SoftAP mode.", 0, 0, 0);
    return FALSE;
  }
}

int main(int argc, char **argv)
{
  fd_set master_fd_set;
  struct sockaddr_storage their_addr;
  char buf[MAX_BUF_LEN];
  int nbytes=0, i;
  socklen_t addr_len;

  /*Initialize the Diag for QXDM logs*/
  if (TRUE != Diag_LSM_Init(NULL))
  {
     mcm_atcop_svc_LOG("Diag_LSM_Init failed !!");
  }

  /* Do not start if we are in DUN+SoftAP mode. */
  if ( IsDunSoftAPMode() )
  {
    LOG_MSG_ERROR("Cannot continue.", 0, 0, 0);
    exit(0);
  }

  /* Register the sighandlers, so the app may be shutdown with a
     kill command.*/
  signal(SIGTERM, sighandler);
  signal(SIGINT, sighandler);

  atcop_exit.srv_id = MCM_ATCOP_V01;
  atcop_exit.srv_exit_func = mcm_atcop_exit_func;
  mcm_ipc_srv_mgr_start(&atcop_exit);

  /* Initialize ATCOP Service */
  if ( mcm_atcop_svc_init() == MCM_ERROR_INTERNAL_V01 )
    return -1;

  while(1)
  {
    master_fd_set = os_params.fds;
    if (select(os_params.max_fd+1, &master_fd_set, NULL, NULL, NULL) == -1)
    {
      LOG_MSG_ERROR("Error in select, errno:%d", errno, 0, 0);
      if( errno == EINTR )
       continue;
      else
       return -1;
    }

    LOG_MSG_INFO1(" Received ATCOP request ..", 0, 0, 0);
    for (i = 0; i <= os_params.max_fd; i++)
    {
      if (FD_ISSET(i, &master_fd_set))
      {
        LOG_MSG_INFO1("\n\nReceived QMI Message\n\n",0,0,0);
        qmi_csi_handle_event(mcm_atcop_svc_state.service_handle, &os_params);
      }
    }
  }
  qmi_csi_unregister(mcm_atcop_svc_state.service_handle);

  LOG_MSG_INFO1("MCM ATCOP service exiting", 0, 0, 0);
  return 0;

}

/*==================================================================
FUNCTION mcm_atcop_exit_func

DESCRIPTION
Used by ril framework to make server exit gracefully

PARAMETERS
  None

RETURN VALUE
  None

DEPENDENCIES
  None

SIDE EFFECTS
  None
====================================================================*/
void mcm_atcop_exit_func()
{
  printf(" mcm_atcop_exit_func ENTER ",0,0,0);
  /* Close the Opened file on EXIT*/
  if( dev_fd > 0 )
  {
    close(dev_fd);
    dev_fd = -1;
  }
  qmi_csi_unregister(mcm_atcop_svc_state.service_handle);
  printf(" mcm_atcop_exit_func EXIT ",0,0,0);

  exit(0);

}

/*===========================================================================
  FUNCTION mcm_atcop_svc_init()

  DESCRIPTION
    Register the MCM ATCOP service with QMUX for all applicable QMI links.

  PARAMETERS
    None

  RETURN VALUE
    None

  DEPENDENCIES
    None

  SIDE EFFECTS
    None
===========================================================================*/
int mcm_atcop_svc_init
(
  void
)
{
  qmi_csi_error         rc;
  int len;
  int val, rval;
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

  LOG_MSG_INFO1("MCM ATCOP service init %d", mcm_atcop_svc_inited,0,0);

  if (!mcm_atcop_svc_inited)
    mcm_atcop_svc_inited = TRUE;
  else
  {
    LOG_MSG_INFO1("MCM ATCOP service already initialized",0,0,0);
    return MCM_ATCOP_ENOERROR;
  }

  /*-------------------------------------------------------------------------
    Register MCM ATCOP service with QCSI
  -------------------------------------------------------------------------*/
  rc = qmi_csi_register
       (
        mcm_atcop_get_service_object_v01(),
        mcm_atcop_svc_hdlr_client_connect_cb,
        mcm_atcop_svc_hdlr_client_disconnect_cb,
        mcm_atcop_svc_hdlr_client_req_cb,
        &mcm_atcop_svc_state,
        &os_params,
        &mcm_atcop_svc_state.service_handle
       );

  if(rc != QMI_CSI_NO_ERR)
  {
    LOG_MSG_ERROR("Unable to register MCM ATCOP service! Error %d", rc,0,0);
    ds_assert(0);
    return MCM_ATCOP_ENOERROR;
  }

  LOG_MSG_INFO1("MCM ATCOP service registered with QCSI Framework",0,0,0);

  if ((dev_fd = open(MCM_ATCOP_DEV_FILE, O_RDWR)) < 0)
  {
    LOG_MSG_ERROR(" Unable to open the device file (/dev/smd8)",0,0,0);
    return MCM_ERROR_INTERNAL_V01;
  }

  LOG_MSG_INFO1("Successfully completed initialization",0,0,0);
  return MCM_ATCOP_ENOERROR;
} /* mcm_mobileap_svc_init */

static qmi_csi_cb_error mcm_atcop_svc_hdlr_client_connect_cb
(
  qmi_client_handle          client_handle,
  void                      *service_handle,
  void                     **connection_handle
)
{
  mcm_atcop_svc_client_info_type *clnt_info_ptr;
  mcm_atcop_svc_state_info_type *atcop_svc_ptr;
  unsigned int     index;
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  ds_assert(connection_handle != NULL);
  ds_assert(service_handle != NULL);

  atcop_svc_ptr = (mcm_atcop_svc_state_info_type *) service_handle;

  for (index=0; index < MCM_ATCOP_SVC_MAX_CLIENT_HANDLES; index++)
  {
    if (NULL == atcop_svc_ptr->client_handle_list[index])
    {
      LOG_MSG_INFO1("\n mcm_atcop_svc_hdlr_client_connect_cb: index=%d\n",index,0,0);
      clnt_info_ptr = (mcm_atcop_svc_client_info_type *)malloc(
                                   sizeof(mcm_atcop_svc_client_info_type));

      if(!clnt_info_ptr)
      {
        LOG_MSG_INFO1("mcm_atcop_svc_hdlr_client_connect_cb: Out of mem",0,0,0);
        return QMI_CSI_CB_CONN_REFUSED;
      }
      else
        break;
     }
  }

  if (index == MCM_ATCOP_SVC_MAX_CLIENT_HANDLES)
  {
    LOG_MSG_ERROR("Client handle slots exhausted", 0, 0, 0);
    return QMI_CSI_CB_NO_MEM;
  }

  memset(clnt_info_ptr, 0, sizeof(mcm_atcop_svc_client_info_type));
  clnt_info_ptr->clnt = client_handle;
  atcop_svc_ptr->num_clients++;
  atcop_svc_ptr->client_handle_list[index] = *connection_handle = clnt_info_ptr;
  LOG_MSG_INFO1("mcm_atcop_svc_hdlr_client_connect_cb: Alloc client 0x%p",
  atcop_svc_ptr->client_handle_list[index],0,0);

  return QMI_CSI_CB_NO_ERR;
}


static void mcm_atcop_svc_hdlr_client_disconnect_cb
(
  void                      *connection_handle,
  void                      *service_handle
)
{
  mcm_atcop_svc_state_info_type  *atcop_svc_ptr;
  unsigned int                   client_index;
  mcm_atcop_svc_client_info_type  *clnt_info;
  mcm_error_t_v01 mcm_err_num = MCM_SUCCESS_V01;
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

  ds_assert(service_handle != NULL);
  atcop_svc_ptr = (mcm_atcop_svc_state_info_type *)service_handle;

  for (client_index=0;client_index < MCM_ATCOP_SVC_MAX_CLIENT_HANDLES; client_index++)
  {
    if (atcop_svc_ptr->client_handle_list[client_index] == connection_handle)
      break;
  }
  if (client_index == MCM_ATCOP_SVC_MAX_CLIENT_HANDLES)
  {
    LOG_MSG_ERROR("mcm_atcop_svc_hdlr_client_disconnect_cb: Invalid Handle %p",
                   connection_handle,0,0);
    return;
  }

  clnt_info = (mcm_atcop_svc_client_info_type *)atcop_svc_ptr->client_handle_list[client_index];
  atcop_svc_ptr->num_clients--;
  LOG_MSG_INFO1("mcm_atcop_svc_hdlr_client_disconnect_cb: Releasing client %p, num_clients: %d",
                 atcop_svc_ptr->client_handle_list[client_index], atcop_svc_ptr->num_clients,0);
  free(atcop_svc_ptr->client_handle_list[client_index]);

  atcop_svc_ptr->client_handle_list[client_index] = NULL;
  return ;
}

static qmi_csi_cb_error mcm_atcop_svc_hdlr_client_req_cb
(
  void                     *connection_handle,
  qmi_req_handle           req_handle,
  unsigned int             msg_id,
  void                     *req_c_struct,
  unsigned int             req_c_struct_len,
  void                     *service_handle
)
{
  qmi_csi_cb_error               rc;
  mcm_atcop_svc_client_info_type *clnt_info;
  mcm_atcop_svc_state_info_type  *atcop_svc_ptr;
  unsigned int                   client_index;
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

  ds_assert(connection_handle != NULL);
  ds_assert(service_handle != NULL);

  rc = QMI_CSI_CB_INTERNAL_ERR;
  clnt_info = (mcm_atcop_svc_client_info_type*)connection_handle;
  atcop_svc_ptr = (mcm_atcop_svc_state_info_type*) service_handle;

  LOG_MSG_INFO1("\nEntered mcm_atcop_svc_hdlr_client_req_cb",0,0,0);

  for (client_index=0;client_index < MCM_ATCOP_SVC_MAX_CLIENT_HANDLES; client_index++)
  {
    if(atcop_svc_ptr->client_handle_list[client_index] == clnt_info)
    {
      LOG_MSG_INFO1("Found the client handle.",0,0,0);
      break;
    }
  }
  if (client_index == MCM_ATCOP_SVC_MAX_CLIENT_HANDLES)
  {
    LOG_MSG_INFO1("mcm_atcop_svc_hdlr_client_req_cb: Invalid clnt handle %p",
                  clnt_info,0,0);
    return rc;
  }

  if((msg_id - MCM_ATCOP_REQ_V01) < (sizeof(req_handle_table) / sizeof(*req_handle_table)))
  {
    if(req_handle_table[msg_id - MCM_ATCOP_REQ_V01])
    {
      rc = req_handle_table[msg_id - MCM_ATCOP_REQ_V01] (clnt_info, req_handle, msg_id,
                                              req_c_struct, req_c_struct_len, service_handle);
    }
    else
    {
      LOG_MSG_INFO1("mcm_atcop_svc_hdlr_client_req_cb: NULL message ID handler: %d",
                    (msg_id - MCM_ATCOP_REQ_V01),0,0);
    }
  }
  else
  {
    LOG_MSG_INFO1("mcm_atcop_svc_hdlr_client_req_cb: Invalid message ID: %d",
                 (msg_id - MCM_ATCOP_REQ_V01),0,0);
  }
  return rc;
}


/*===========================================================================
   FUNCTION MCM_ATCOP_HANDLE_REQ()

   DESCRIPTION
              Handle the ATCOP request from the client.

   PARAMETERS
     sp:          MCM atcop service instance state pointer for this qmi link
     cmd_buf_p:   Message Header
     cl_sp:       Coresponding client state pointer
     sdu_in:      input command data

   RETURN VALUE
     dsm * pointing to the response to be sent to host
     NULL if no response was generated or command was discarded

   DEPENDENCIES
     qmi_atcop_msgr_init() must have been called

   SIDE EFFECTS
     None
 ===========================================================================*/

static qmi_csi_cb_error  mcm_atcop_handle_req
(
  mcm_atcop_svc_client_info_type        *clnt_info,
  qmi_req_handle           req_handle,
  int                      msg_id,
  void                    *req_c_struct,
  int                      req_c_struct_len,
  void                     *service_handle
)
{
  mcm_atcop_svc_state_info_type  *atcop_sp;
  mcm_atcop_req_msg_v01 *req_ptr;
  mcm_atcop_resp_msg_v01 resp_msg;
  unsigned int index;
  mcm_error_t_v01 mcm_err_num = MCM_SUCCESS_V01;
  int  nwrote, nbytes, result, i, ret;
  char *token = NULL, *save_ptr = NULL;
  char temp_buff[MCM_ATCOP_MAX_REQ_MSG_SIZE_V01] = {0};
  char temp_buff1[MCM_ATCOP_MAX_REQ_MSG_SIZE_V01] = {0};
  char no_echo_req[MCM_ATCOP_MAX_REQ_MSG_SIZE_V01] = {0};
  char no_echo_resp[MCM_ATCOP_MAX_RESP_MSG_SIZE_V01] = {0};

  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

  ds_assert(req_c_struct != NULL);
  ds_assert(req_c_struct_len > 0);
  ds_assert(clnt_info != NULL);
  ds_assert(service_handle != NULL);

  atcop_sp = (mcm_atcop_svc_state_info_type *)service_handle;
  req_ptr = (mcm_atcop_req_msg_v01 *)req_c_struct;

  LOG_MSG_INFO1 ("Client %p got ATCOP request. ", clnt_info,0,0);

  memset(&resp_msg, 0, sizeof(mcm_atcop_resp_msg_v01));

  /* ATCOP Command processing. */
  if( req_ptr->cmd_len < MCM_ATCOP_AT_CMD_LEN )
  {
    resp_msg.resp.result = MCM_RESULT_FAILURE_V01;
    resp_msg.resp.error = MCM_ERROR_BADPARM_V01;
    LOG_MSG_ERROR("Invalid ATCOP command length %d ", req_ptr->cmd_len, 0, 0);
    qmi_csi_send_resp(req_handle, msg_id, &resp_msg,
                                      sizeof(mcm_atcop_resp_msg_v01));
    return QMI_CSI_CB_NO_ERR;
  }

  if ( !first_client )
  {

    strlcpy(no_echo_req, MCM_ATCOP_ECHO_DISABLE, MCM_ATCOP_ECHO_DISABLE_LEN);

    result = mcm_atcop_process_req( no_echo_req, strlen(no_echo_req), no_echo_resp);
    if ( result )
    {
      resp_msg.resp.result = MCM_RESULT_FAILURE_V01;
      resp_msg.resp.error = result;
      LOG_MSG_ERROR("Unable to process ATCOP request result: %d ", result, 0, 0);
      qmi_csi_send_resp(req_handle, msg_id, &resp_msg,
      sizeof(mcm_atcop_resp_msg_v01));
      return QMI_CSI_CB_NO_ERR;
    }
    first_client = 1;
  }

  if(strstr (req_ptr->cmd_req,MCM_ATCOP_CMD_DELIMITER))
  {
    memcpy(&temp_buff, req_ptr->cmd_req, MCM_ATCOP_MAX_REQ_MSG_SIZE_V01);

    token = strtok_r(temp_buff, MCM_ATCOP_CMD_DELIMITER, &save_ptr);
    strlcpy(temp_buff1, token, MCM_ATCOP_MAX_REQ_MSG_SIZE_V01);

    /* walk through other tokens */
    while( token != NULL )
    {
      ret = atcop_cmd_validate(temp_buff1);
      if ( ret )
      {
        resp_msg.resp.result = MCM_RESULT_FAILURE_V01;
        resp_msg.resp.error = ret;
        qmi_csi_send_resp(req_handle, msg_id, &resp_msg,
                                         sizeof(mcm_atcop_resp_msg_v01));
        return QMI_CSI_CB_NO_ERR;
      }
      token = strtok_r(NULL, MCM_ATCOP_CMD_DELIMITER, &save_ptr);
      if(token)
      {
        memset(temp_buff1, 0, sizeof(temp_buff1));
        strlcpy(temp_buff1, MCM_ATCOP_AT_CMD, sizeof(temp_buff1));
        strlcat(temp_buff1, token, sizeof(temp_buff1));
      }
    }
  }
  else
  {
    ret = atcop_cmd_validate(req_ptr->cmd_req);
    if ( ret )
    {
      resp_msg.resp.result = MCM_RESULT_FAILURE_V01;
      resp_msg.resp.error = ret;
      qmi_csi_send_resp(req_handle, msg_id, &resp_msg,
                                       sizeof(mcm_atcop_resp_msg_v01));
      return QMI_CSI_CB_NO_ERR;
    }
  }
  result = mcm_atcop_process_req( req_ptr->cmd_req, req_ptr->cmd_len, &(resp_msg.cmd_resp));
  if (result && (result != MCM_ERROR_INSUFFICIENT_RESOURCES_V01))
  {
    resp_msg.resp.result = MCM_RESULT_FAILURE_V01;
    resp_msg.resp.error = result;
    LOG_MSG_ERROR("Unable to process ATCOP request result: %d ", result, 0, 0);
    qmi_csi_send_resp(req_handle, msg_id, &resp_msg,
                                     sizeof(mcm_atcop_resp_msg_v01));
    return QMI_CSI_CB_NO_ERR;
  }
  resp_msg.resp_len = strlen(resp_msg.cmd_resp);
  resp_msg.resp.error = result;
  resp_msg.resp.result = MCM_RESULT_SUCCESS_V01;
  resp_msg.cmd_resp_valid = TRUE;
  resp_msg.resp_len_valid = TRUE;

  LOG_MSG_INFO1 ("Successfully processed ATCOP request. ", 0,0,0);
  qmi_csi_send_resp(req_handle, msg_id, &resp_msg,
                                   sizeof(mcm_atcop_resp_msg_v01));

  return QMI_CSI_CB_NO_ERR;

}

int mcm_atcop_process_req(char *cmd_req, int cmd_len, char *cmd_resp)
{
  int ret, filled = 0, nbytes, nwrote = 0, i, response_overflow_check = 0;
  char buffer[MAX_BUF_LEN] = {0};
  fd_set master_fd_set;
  struct timeval tv;
  char match_copn[] = "+COPN";
  int index = 0;
  char *last_copn_entry = NULL;

  if ( !cmd_req || !cmd_resp)
  {
    LOG_MSG_ERROR("Invalid ATCOP command ", 0, 0, 0);
    return MCM_ERROR_BADPARM_V01;
  }

  /* Set the MAX timeout for read to 5 mins */
  tv.tv_sec = MCM_ATCOP_MAX_READ_TIMEOUT;
  tv.tv_usec = 0;


  /*
   Below are the steps to be followed for validation:

   1. Each command should have a minimum of 2 characters
   2. Each command is separated by ;
   3. From the second command onwards AT should not be given.

 */

  strlcat(cmd_req, MCM_ATCOP_ESC_CHARS, MCM_ATCOP_MAX_REQ_MSG_SIZE_V01);

  /* Write the requested command onto the device file. */
  nwrote = write(dev_fd, cmd_req, strlen(cmd_req));
  if( nwrote < 0 )
  {
    LOG_MSG_ERROR(" ATCOP Command write to device file (/dev/smd8) failed.",0,0,0);
    return MCM_ERROR_INTERNAL_V01;
  }
  FD_ZERO(&master_fd_set);
  FD_SET(dev_fd, &master_fd_set);

  do
  {
    memset(buffer, 0, sizeof(buffer));
    if ((ret = select(dev_fd+1, &master_fd_set, NULL, NULL, &tv)) == -1)
    {
      LOG_MSG_ERROR("Error in select, errno:%d", errno, 0, 0);
      if( errno == EINTR )
       continue;
      else
       return -1;
    }
    /* No more data available after 5 mins of waiting. Hence return with whatever obtained. */
    else if (ret == 0)
    {
      LOG_MSG_INFO1("Select got timed out after 5 min wait."
                    " Return with whatever obtained.", 0, 0, 0);
      return MCM_SUCCESS_V01;
    }
    if((nbytes = read(dev_fd, (uint8 *)(&(buffer)), MAX_BUF_LEN - 1)) < 0 )
    {
      LOG_MSG_ERROR("ATCOP Command response reception failed. errno: %d ", errno,0,0);
      return MCM_ERROR_INTERNAL_V01;
    }

    if ((filled + nbytes) <= (MCM_ATCOP_MAX_RESP_MSG_SIZE_V01 - 1))
    {
      memcpy(cmd_resp + filled, buffer, nbytes);
    }
    else
    {
      /* Fill up the remaining buffer space. */
      memcpy(cmd_resp + filled, buffer, ((MCM_ATCOP_MAX_RESP_MSG_SIZE_V01 - 1) - filled));
      /* This flag will be used to flush off the unsent data.*/
      response_overflow_check = 1;
      break;
    }

    filled += nbytes;

    if (check_trailing_chars(filled, cmd_resp))
    {
      break;
    }
  }while(1);

  /* If we have completely filled the response buffer of 4K and if there is
     still data left, we flush the read pipe, by keep on reading till the
     mandatory trailing charaters are reached.
   */
  if (response_overflow_check)
  {
    do
    {
      memset(buffer, 0, sizeof(buffer));
      if ((nbytes = read(dev_fd, (uint8 *)(&(buffer)), MAX_BUF_LEN - 1)) < 0)
      {
        LOG_MSG_ERROR("ATCOP Command response reception failed. errno: %d ",
                      errno,0,0);
        return MCM_ERROR_INTERNAL_V01;
      }

      if (check_trailing_chars(nbytes, buffer))
      {
        LOG_MSG_INFO1("Emptied the remaining data. Success", 0, 0, 0);
        break;
      }
    }while(1);
  }

  /* Convert to upper case for ease of comparision. */
  str_to_upcase(cmd_req);

  /* Whenever there is a overflow in buffer and if the request command as +COPN,
     we will be checking for the completeness of the entry.
   */
  if (response_overflow_check && (strlen(cmd_req) >= MCM_ATCOP_COPN_CMD_LEN) &&
     (strstr(cmd_req, match_copn)))
  {
    if((strlen(cmd_resp) != 0) && cmd_resp[strlen(cmd_resp) - 1] == '\n')
    {
      /* If the entry has ended completely then return from here. */
      return MCM_ERROR_INSUFFICIENT_RESOURCES_V01;
    }
    /* Search for  +COPN string and find the last instance of the same.
       If a valid location is found, add a '\0' appropriately to discard
       the last incmplete entry.
     */
    if ((last_copn_entry = findlast(cmd_resp, match_copn)) && (last_copn_entry > cmd_resp))
    {
      index = last_copn_entry - cmd_resp;
    }
    else
    {
      /* Should not come here in ideal cases. */
      return MCM_SUCCESS_V01;
    }
    LOG_MSG_INFO1("Original response length: %d new resp len: %d",
                  strlen(cmd_resp), index, 0);

    *(cmd_resp + index) = '\0';
    return MCM_ERROR_INSUFFICIENT_RESOURCES_V01;
  }
  else if(response_overflow_check)
  {
    /* If we are sending truncated response and if the command is not AT+COPN,
       return with MCM_ERROR_INSUFFICIENT_RESOURCES_V01 so that the client
       can be notified accordingly.
     */
    return MCM_ERROR_INSUFFICIENT_RESOURCES_V01;
  }
  return MCM_SUCCESS_V01;
}

/* Check for the trailing characters to break. */
int check_trailing_chars(int filled, char *cmd_resp)
{
  if (cmd_resp == NULL)
  {
    LOG_MSG_ERROR(" Invalid arguments ", 0, 0, 0);
    return 1;
  }
  if ((( filled >= MCM_OK_STRING_LEN ) && ( !(strncasecmp(
       (unsigned char *)(cmd_resp + (filled - MCM_OK_STRING_LEN)),
        MCM_ATCOP_RESP_OK, MCM_OK_STRING_LEN)))) ||

      (( filled >= MCM_ERROR_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp + (filled - MCM_ERROR_STRING_LEN)),
         MCM_ATCOP_RESP_ERROR1, MCM_ERROR_STRING_LEN )))) ||

      (( filled >= MCM_CME_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp), MCM_ATCOP_RESP_CME_ERROR,
         MCM_CME_STRING_LEN )))) ||

      (( filled >= MCM_CME_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp), MCM_ATCOP_RESP_CMS_ERROR,
         MCM_CME_STRING_LEN )))) ||

      (( filled >= MCM_ERROR_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp), MCM_ATCOP_RESP_ERROR2,
         MCM_ERROR_STRING_LEN )))) ||

      (( filled >= MCM_CONNECT_STRING_LEN) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp),
         MCM_ATCOP_RESP_CONNECT1, MCM_CONNECT_STRING_LEN)))) ||

      (( filled >= MCM_CONNECT_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp + (filled - MCM_CONNECT_STRING_LEN)),
         MCM_ATCOP_RESP_CONNECT2, MCM_CONNECT_STRING_LEN )))) ||

      (( filled >= MCM_CME_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp),
         MCM_ATCOP_RESP_NO_CARRIER1, MCM_CME_STRING_LEN )))) ||

      (( filled >= MCM_CME_STRING_LEN ) && ( !(strncasecmp(
       ( unsigned char *)(cmd_resp + (filled - MCM_CME_STRING_LEN)),
         MCM_ATCOP_RESP_NO_CARRIER2, MCM_CME_STRING_LEN )))))
    return 1;
  /* If it is not the end of the response. */
  return 0;
}
