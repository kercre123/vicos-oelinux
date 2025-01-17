/*!
  @file
  atfwd_daemon.c

  @brief
  ATFWD daemon which registers with QMI ATCOP service and forwards AT commands

*/

/*===========================================================================

  Copyright (c) 2012, 2015 Qualcomm Technologies, Inc. All Rights Reserved

  Qualcomm Technologies Proprietary

  Export of this technology or software is regulated by the U.S. Government.
  Diversion contrary to U.S. law prohibited.

  All ideas, data and information contained in or disclosed by
  this document are confidential and proprietary information of
  Qualcomm Technologies, Inc. and all rights therein are expressly reserved.
  By accepting this material the recipient agrees that this material
  and the information contained therein are held in confidence and in
  trust and will not be used, copied, reproduced in whole or in part,
  nor its contents revealed in any manner to others without the express
  written permission of Qualcomm Technologies, Inc.

===========================================================================*/

/*===========================================================================

                        EDIT HISTORY FOR MODULE

This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.


when       who      what, where, why
--------   ---      ---------------------------------------------------------
04/11/11  jaimel  ATFWD-daemon to register and forward AT commands to Apps
04/19/11  c_spotha Added default port changes

===========================================================================*/

/*===========================================================================

                           INCLUDE FILES

===========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <stdbool.h>
#include <qmi_atcop_srvc.h>
#include <qmi.h>
#include <string.h>
#include <cutils/properties.h>
#include "common_log.h"

#include "qmi_client.h"
#include "qmi_client_instance_defs.h"
#include "qmi_cci_target_ext.h"
#include "qmi_idl_lib.h"
#include "access_terminal_service_v01.h"

#define MAX_DIGITS 10
#define DEFAULT_QMI_PORT QMI_PORT_RMNET_SDIO_0
#define DEFAULT_SMD_PORT QMI_PORT_RMNET_0

#define ATFWD_ATCOP_PORTS                 2
#define ATFWD_MAX_RETRY_ATTEMPTS          5
#define ATFWD_RETRY_DELAY                 5  /* Retry delay in sec */

#define LOGI(...) fprintf(stderr, "I:" __VA_ARGS__)

typedef enum {
    INIT_QMI = 0,
    INIT_QMI_SRVC,
    INIT_ATFWD_SRVC,
    INIT_MAX
} atfwd_init_type_t;

typedef struct {
    int opcode;
    char *name;
    int ntokens;
    char **tokens;
} AtCmd;

typedef struct {
    int result;
    char *response;
}AtCmdResponse;

/*qmi message library handle*/
int qmiHandle = QMI_INVALID_CLIENT_HANDLE;

/*===========================================================================

                           Global Variables

===========================================================================*/

int qmiErrorCode; //Stores the QMI error codes
int userHandle; //Connection ID
int userHandleSMD; //Conn ID for SVLTE II , USB --> SMD --> 8k modem
qmi_atcop_abort_type abortType; //AT command abort type
const char *qmiPort = NULL;
const char *secondaryPort = NULL;
qmi_atcop_at_cmd_fwd_req_type atCmdFwdReqType[] = {
    { //AT command fwd type
        1, // Number of commands
        {
            { QMI_ATCOP_AT_CMD_NOT_ABORTABLE, "+CFUN"},
        }
    },
    { //AT command fwd type
        1, // Number of commands
        {
            { QMI_ATCOP_AT_CMD_NOT_ABORTABLE, "$QCPWRDN"},
        }
    },
};

qmi_idl_service_object_type qmi_at_svc_obj;
qmi_client_type             qmi_at_svc_client;
qmi_client_os_params        qmi_at_os_params;
qmi_client_type             qmi_at_notifier;
#define QMI_AT_CLIENT_INIT_TIMEOUT 4
#define MAX_QMI_AT_SVC_CONNECT_ATTEMPTS 2
#define QMI_ATFWD_SYNC_MSG_TIMEOUT 10000
#define QMI_AT_MAX_REQ_MSG_SIZE 512
#define QMI_AT_MAX_RESP_MSG_SIZE 512

//=============================================
// QMI AT RAW message helper functions

extern int qmi_atcop_reg_at_command_fwd_req_msg_helper
(
  qmi_atcop_at_cmd_fwd_req_type     *cmd_fwd_req,
  unsigned char                     *msg,
  int                               *msg_size
);

extern void qmi_atcop_srvc_indication_cb_helper
(
  unsigned long         msg_id,
  unsigned char         *rx_msg_buf,
  int                   rx_msg_len,
  qmi_atcop_indication_data_type    *ind_data;
);

extern int qmi_atcop_fwd_at_cmd_resp_helper
(
  qmi_atcop_fwd_resp_at_resp_type   *at_resp,
  int                               *qmi_err_code,
  unsigned char                     *msg,
  int                               *msg_size
);

//=============================================

qmi_atcop_at_cmd_hndl_type commandHandle; //ATFWD request handle
qmi_atcop_at_cmd_fwd_ind_type request; //Input request string

//qmi_atcop_fwd_resp_status_type responseStatus; //ATFWD response status
qmi_atcop_fwd_resp_result_type responseResult; //ATFWD response result
qmi_atcop_fwd_resp_response_type responseType; //ATFWD response type
qmi_atcop_fwd_resp_at_resp_type atCmdResponse; //Actual ATFWD response

AtCmd fwdcmd;
AtCmdResponse fwdresponse;

pthread_cond_t ctrCond; //Condition variable that will be used to indicate if a request has arrived
pthread_mutex_t ctrMutex; //Mutex that will be locked when a request is processed
int newRequest = 0; //New request indication
int regForPrimaryPort = 0; //Re registration indication for primary qmi port
int regForSecondaryPort = 0; //Re registration indication for secondary qmi port

typedef struct atfwd_sig_handler_s
{
  int sig;
  __sighandler_t handler;
} atfwd_sig_handler_t;

/* All termination SIGNALS except SIGKILL
 * SIGKILL cannot be handled or ignored
*/
atfwd_sig_handler_t atfwd_sig_handler_tbl[] =
{
  {SIGTERM, NULL},
  {SIGINT, NULL},
  {SIGQUIT, NULL},
  {SIGHUP, NULL}
};

/*===========================================================================
  FUNCTION  parseInput
===========================================================================*/
/*!
@brief
  Parses the input request string and populates the AtCmd struct with the
  data to forward

@return
  None

*/
/*=========================================================================*/
void parseInput()
{
    int i;
    fwdcmd.opcode = request.op_code;
    fwdcmd.name = strdup((char *)request.at_name);
    fwdcmd.ntokens = request.num_tokens;
    fwdcmd.tokens = calloc(request.num_tokens, sizeof(char *));
    if(NULL != fwdcmd.tokens) {
        for (i = 0; i < request.num_tokens; i++) {
            fwdcmd.tokens[i] = strdup((char *)request.tokens[i]);
        }
    }
}

/*===========================================================================
  FUNCTION  exitDaemon
===========================================================================*/
/*!
@brief
  Utility method which handles when user presses CTRL+C

@return
  None

@note
  None
*/
/*=========================================================================*/
void exitDaemon(int sig)
{
    LOGI("Going to kill ATFWD daemon\n");
    (void)sig;
    unsigned int i=0;
    /* Note that the handler should ignore all the reg. signals
     * because they do not want to be interfered
     * while an ongoing signal is being processed
     */
    for(i=0; i<sizeof(atfwd_sig_handler_tbl)/sizeof(atfwd_sig_handler_t); i++) {
        signal(atfwd_sig_handler_tbl[i].sig, SIG_IGN);
    }
    int clientRelease = qmi_atcop_srvc_release_client (userHandle, &qmiErrorCode);
    if (clientRelease < 0) {
        LOGI("QMI client release error: %d\n", qmiErrorCode);
    }
    if (qmiHandle >= 0) {
        qmi_release(qmiHandle);
    }
    pthread_cond_destroy(&ctrCond);
    pthread_mutex_destroy(&ctrMutex);

    for(i=0; i<sizeof(atfwd_sig_handler_tbl)/sizeof(atfwd_sig_handler_t); i++) {
        LOGI("\natfwd_sig_handler_tbl[i].sig : %d", atfwd_sig_handler_tbl[i].sig);
        if (atfwd_sig_handler_tbl[i].sig == sig &&
            atfwd_sig_handler_tbl[i].handler != NULL) {
            /* call  default installed handler */
            LOGI("\ncall default handler [%p] for sig [%d]",
                  atfwd_sig_handler_tbl[i].handler,
                  atfwd_sig_handler_tbl[i].sig);
            (atfwd_sig_handler_tbl[i].handler)(sig);
            break;
        }
    }
    exit(0);
}

/*===========================================================================
  FUNCTION:  signalInit
===========================================================================*/
/*!
    @brief
    Signal specific initialization

    @return
    void
*/
/*=========================================================================*/
void signalInit(void)
{
    unsigned int i=0;
    __sighandler_t temp;

    for(i=0; i<sizeof(atfwd_sig_handler_tbl)/sizeof(atfwd_sig_handler_t); i++) {
        temp = atfwd_sig_handler_tbl[i].handler;
        atfwd_sig_handler_tbl[i].handler = signal(atfwd_sig_handler_tbl[i].sig,
                                                  exitDaemon);
        /* swap previous handler back if signal() was unsuccessful */
        if (SIG_ERR == atfwd_sig_handler_tbl[i].handler) {
            atfwd_sig_handler_tbl[i].handler = temp;
        }
    }
}

/*===========================================================================
  FUNCTION  is_supported_qcci

  From 8996 onwards ATFWD Daemon will work using QCCI. All earlier targets
  use legacy QCCI.

===========================================================================*/
bool is_supported_qcci(void)
{
    return true;
}

/*===========================================================================
  FUNCTION  sendSuccessResponse
===========================================================================*/
/*!
@brief
  Sends OK response to QMI.

@return
  None

@note
  None
*/
/*=========================================================================*/
void sendSuccessResponse()
{
    //responseStatus = QMI_ATCOP_SUCCESS;
    responseResult = QMI_ATCOP_RESULT_OK;
    responseType = QMI_ATCOP_RESP_COMPLETE;
    atCmdResponse.at_hndl = commandHandle;
    //atCmdResponse.status = responseStatus;
    atCmdResponse.result = responseResult;
    atCmdResponse.response = responseType;
    atCmdResponse.at_resp = NULL;
    if (qmi_atcop_fwd_at_cmd_resp(userHandle, &atCmdResponse, &qmiErrorCode) < 0) {
        LOGI("QMI response error: %d\n", qmiErrorCode);
    }
}

/*===========================================================================
  FUNCTION  sendResponse
===========================================================================*/
/*!
@brief
  Sends response to QMI.

@return
  None

@note
  None
*/
/*=========================================================================*/
void sendResponse(AtCmdResponse *response)
{
    unsigned char resp_msg[QMI_AT_MAX_RESP_MSG_SIZE];
    unsigned char qmi_msg_resp_buf[QMI_AT_MAX_RESP_MSG_SIZE];
    int resp_msg_len = 0;
    int qmi_msg_resp_buf_len =0;
    qmi_client_error_type qmi_err = QMI_NO_ERR;

    //responseStatus = QMI_ATCOP_SUCCESS;
    responseResult = QMI_ATCOP_RESULT_OK;
    responseType = QMI_ATCOP_RESP_COMPLETE;
    atCmdResponse.at_hndl = commandHandle;
    //atCmdResponse.status = responseStatus;
    atCmdResponse.result = responseResult;
    atCmdResponse.response = responseType;
    if ((response->result != QMI_ATCOP_RESULT_OK) && (request.cmee_val == 0)) {
        atCmdResponse.at_resp = NULL;
    } else {
        char *msg = NULL;
        unsigned long s3 = request.s3_val;
        unsigned long s4 = request.s4_val;
        atCmdResponse.at_resp = (unsigned char *)msg;
    }
    if ( !is_supported_qcci() )
    {
        if (qmi_atcop_fwd_at_cmd_resp(userHandle, &atCmdResponse, &qmiErrorCode) < 0) {
            LOGI("QMI response error: %d\n", qmiErrorCode);
        }
    }
    else
    {
        memset(resp_msg, 0x00, QMI_AT_MAX_RESP_MSG_SIZE);
        memset(qmi_msg_resp_buf, 0x00, QMI_AT_MAX_RESP_MSG_SIZE);
        qmi_atcop_fwd_at_cmd_resp_helper(&atCmdResponse, &qmiErrorCode, resp_msg, &resp_msg_len);

        LOGI("sending QMI_AT_FWD_RESP_AT_CMD_RESP_V01 message\n");
        qmi_err = qmi_client_send_raw_msg_sync(qmi_at_svc_client,
                            QMI_AT_FWD_RESP_AT_CMD_RESP_V01,
                            resp_msg,
                            sizeof(resp_msg)-resp_msg_len,
                            qmi_msg_resp_buf,
                            sizeof(qmi_msg_resp_buf),
                            &qmi_msg_resp_buf_len,
                            QMI_ATFWD_SYNC_MSG_TIMEOUT);

        LOGI("qmi_client_send_raw_msg_sync returned: %d\n", qmi_err);
    }
    if(NULL != atCmdResponse.at_resp) {
        free(atCmdResponse.at_resp);
        atCmdResponse.at_resp = NULL;
    }
}

/*===========================================================================
  FUNCTION  sendInvalidCommandResponse
===========================================================================*/
/*!
@brief
  Sends ERROR response to QMI.

@return
  None

@note
  None
*/
/*=========================================================================*/
void sendInvalidCommandResponse()
{
    unsigned char resp_msg[QMI_AT_MAX_RESP_MSG_SIZE];
    unsigned char qmi_msg_resp_buf[QMI_AT_MAX_RESP_MSG_SIZE];
    int resp_msg_len = 0;
    int qmi_msg_resp_buf_len =0;
    qmi_client_error_type qmi_err = QMI_NO_ERR;
    responseResult = QMI_ATCOP_RESULT_ERROR;
    responseType   = QMI_ATCOP_RESP_COMPLETE;

    atCmdResponse.at_hndl  = commandHandle;
    atCmdResponse.result   = responseResult;
    atCmdResponse.response = responseType;

    if (request.cmee_val == 0) {
        atCmdResponse.at_resp = NULL;
    } else {
        char *response;
        char s3Val[MAX_DIGITS];
        char s4Val[MAX_DIGITS];

        snprintf(s3Val, MAX_DIGITS, "%c", (char) request.s3_val);
        snprintf(s4Val, MAX_DIGITS, "%c", (char) request.s4_val);

        size_t respLen  = ((strlen(s3Val) * 2) + (strlen(s4Val) * 2) + 13 + 1) * sizeof(char);
        response = (char *)malloc(respLen);
        if (!response) {
            LOGI("No memory for generating invalid command response\n");
            atCmdResponse.at_resp = NULL;
        } else {
            snprintf(response, respLen, "%s%s+CME ERROR :2%s%s", s3Val, s4Val, s3Val, s3Val);
            atCmdResponse.at_resp = (unsigned char *)response;
        }
    }

    if (!is_supported_qcci() )
    {
        if (qmi_atcop_fwd_at_cmd_resp(userHandle, &atCmdResponse, &qmiErrorCode) < 0) {
            LOGI("QMI response error: %d\n", qmiErrorCode);
        }
    }
    else
    {
        memset(resp_msg, 0x00, QMI_AT_MAX_RESP_MSG_SIZE);
        memset(qmi_msg_resp_buf, 0x00, QMI_AT_MAX_RESP_MSG_SIZE);
        qmi_atcop_fwd_at_cmd_resp_helper(&atCmdResponse, &qmiErrorCode, resp_msg, &resp_msg_len);

        LOGI("sending QMI_AT_FWD_RESP_AT_CMD_RESP_V01 message\n");
        qmi_err = qmi_client_send_raw_msg_sync(qmi_at_svc_client,
                            QMI_AT_FWD_RESP_AT_CMD_RESP_V01,
                            resp_msg,
                            sizeof(resp_msg)-resp_msg_len,
                            qmi_msg_resp_buf,
                            sizeof(qmi_msg_resp_buf),
                            &qmi_msg_resp_buf_len,
                            QMI_ATFWD_SYNC_MSG_TIMEOUT);
        LOGI("qmi_client_send_raw_msg_sync returned: %d\n", qmi_err);
    }

    if (atCmdResponse.at_resp) {
        free(atCmdResponse.at_resp);
        atCmdResponse.at_resp = NULL;
    }
}

/*===========================================================================
  FUNCTION  sendCommand
===========================================================================*/
/*!
@brief
  Routine that will be invoked by QMI upon a request for AT command. It
  checks for the validity of the request and finally spawns a new thread to
  process the key press events.

@return
  None

@note
  None

*/
/*=========================================================================*/
static void atCommandCb(int userHandle, qmi_service_id_type serviceID,
                             void *userData, qmi_atcop_indication_id_type indicationID,
                             qmi_atcop_indication_data_type  *indicationData)
{
    LOGI("atCommandCb\n");

    /* Check if it's an abort request */
    if (indicationID == QMI_ATCOP_SRVC_ABORT_MSG_IND_TYPE) {
        LOGI("Received abort message from QMI\n");
    } else if (indicationID == QMI_ATCOP_SRVC_AT_FWD_MSG_IND_TYPE) {
        LOGI("Received AT command forward request\n");
        pthread_mutex_lock(&ctrMutex);
        commandHandle = indicationData->at_hndl;
        request = indicationData->at_cmd_fwd_type;
        parseInput();
        newRequest = 1;
        pthread_cond_signal(&ctrCond);
        pthread_mutex_unlock(&ctrMutex);
    }
}

void initAtCopServiceByPort(const char *port, int *handle) {
    int retryCnt;
    for (retryCnt = 1; retryCnt <= ATFWD_MAX_RETRY_ATTEMPTS; ++retryCnt) {
        *handle = qmi_atcop_srvc_init_client(port, atCommandCb, NULL , &qmiErrorCode);
        LOGI("qmi_atcop_srvc_init_client - QMI Err code %d , handle %d", qmiErrorCode, *handle);
        if (*handle < 0 || qmiErrorCode != 0) {
            LOGI("Could not register with the QMI Interface on port %s. " \
                    "The QMI error code is %d", port, qmiErrorCode);
            // retry after yielding..
            sleep(retryCnt * ATFWD_RETRY_DELAY);
            continue;
        }
        break;
    }
    return;
}

void stopSelf() {
    LOGI("Stop the daemon....");
    return;
}

/*===========================================================================
  FUNCTION  registerATCommands

  Register all AT commands with the QMI AT service.
  Note that client initialization is already done as part of qmi_client_init_
  instance.

===========================================================================*/
bool registerATCommands(void)
{
    int i=0;
    int nCommands=0;
    int num_errors=0;
    qmi_client_error_type qmi_err = QMI_NO_ERR;
    unsigned char req_msg[QMI_AT_MAX_REQ_MSG_SIZE];
    unsigned char resp_msg[QMI_AT_MAX_RESP_MSG_SIZE];
    unsigned int actual_resp_len = 0;
    unsigned int req_msg_len = 0;

    nCommands = sizeof(atCmdFwdReqType) / sizeof(atCmdFwdReqType[0]);
    LOGI("Trying to register %d commands:\n", nCommands);

    for ( i=0; i< nCommands; i++)
    {
        LOGI("cmd%d: %s\n", i, atCmdFwdReqType[i].qmi_atcop_at_cmd_fwd_req_type[0].at_cmd_name);

        memset( req_msg , 0x00, QMI_AT_MAX_REQ_MSG_SIZE);
        memset( resp_msg, 0x00, QMI_AT_MAX_RESP_MSG_SIZE);

        qmi_atcop_reg_at_command_fwd_req_msg_helper(&atCmdFwdReqType[i], req_msg, &req_msg_len);

        LOGI("sending QMI_AT_REG_AT_CMD_FWD_REQ_V01 message\n");
        qmi_err = qmi_client_send_raw_msg_sync(qmi_at_svc_client,
                                    QMI_AT_REG_AT_CMD_FWD_REQ_V01,
                                    req_msg,
                                    sizeof(req_msg)-req_msg_len,
                                    resp_msg,
                                    sizeof(resp_msg),
                                    &actual_resp_len,
                                    QMI_ATFWD_SYNC_MSG_TIMEOUT);

        if ( qmi_err != QMI_NO_ERR)
        {
            LOGI("Could not register AT command : %s with the QMI Interface - Err code:%d\n",
                     atCmdFwdReqType[i].qmi_atcop_at_cmd_fwd_req_type[0].at_cmd_name, qmi_err);
            num_errors++;
            qmi_err = QMI_NO_ERR;
        }
    }

    if ( num_errors == nCommands )
    {
        LOGI("AT commands registration failure... Release client handle.");
        qmi_client_release(qmi_at_svc_client);
        return false;
    }

    LOGI("Registered AT Commands event handler\n");
    return true;
}

void initAtcopServiceAndRegisterCommands(const char *port, int *handle) {
    int i, nErrorCnt, nCommands;
    i = nErrorCnt = 0;

    initAtCopServiceByPort(port, handle);

    if (*handle > 0) {
        nCommands = sizeof(atCmdFwdReqType) / sizeof(atCmdFwdReqType[0]);
        LOGI("Trying to register %d commands:\n", nCommands);
        for (i = 0; i < nCommands ; i++) {
            LOGI("cmd%d: %s\n", i, atCmdFwdReqType[i].qmi_atcop_at_cmd_fwd_req_type[0].at_cmd_name);

            qmiErrorCode = 0;
            int registrationStatus = qmi_atcop_reg_at_command_fwd_req(*handle, \
                                              &atCmdFwdReqType[i], &qmiErrorCode);
            LOGI("qmi_atcop_reg_at_command_fwd_req: %d", qmiErrorCode);
            if (registrationStatus < 0 || qmiErrorCode != 0) {
                LOGI("Could not register AT command : %s with the QMI Interface - Err code:%d\n",
                     atCmdFwdReqType[i].qmi_atcop_at_cmd_fwd_req_type[0].at_cmd_name, qmiErrorCode);
                nErrorCnt++;
                qmiErrorCode = 0;
            }
        }

        if(nErrorCnt == nCommands) {
            LOGI("AT commands registration failure... Release client handle: %d\n", *handle);
            qmi_atcop_srvc_release_client(*handle, &qmiErrorCode);
            *handle = -1;
            return;
        }
    } else {
        LOGI("ATcop Service Init failed\n");
        return;
    }

    LOGI("Registered AT Commands event handler\n");
    return;
}

/*===========================================================================
  FUNCTION  at_fwd_sys_down_evt_handlers

  Call back to be registered with qmi_client_register_error_cb.
  When service terminates on the modem, release qmi_client.

===========================================================================*/
static void at_fwd_sys_down_evt_handler
(
  qmi_client_type       clnt,
  void                 *error_cb_data
)
{
  int srv_id = (intptr_t) error_cb_data;
  qmi_client_error_type client_err = QMI_NO_ERR;

  LOGI("ATFWD sys down event recvd. srv_id:%d", srv_id);

  client_err = qmi_client_release(qmi_at_svc_client);
  qmi_at_svc_client = NULL;
  LOGI("QMI AT client released with err code: %d", (int) client_err);
}

/*===========================================================================
  FUNCTION  at_fwd_sys_up_handler

  Call back to be registered with qmi_client_notifier_init .
  When service is available back on the modem, initiate a new client
  registration.

===========================================================================*/
static void at_fwd_sys_up_handler
(
 qmi_client_type                user_handle,
 qmi_idl_service_object_type    service_obj,
 qmi_client_notify_event_type   service_event,
 void                           *notify_cb_data
)
{
    LOGI("Recvd QMI AT notification. service_event: %d", (int) service_event);
    switch ( service_event)
    {
    case QMI_CLIENT_SERVICE_COUNT_INC:
        LOGI("QMI AT service is back up.", (int) service_event);
        // There is only one port when using QMI_CLIENT_INIT_INSTANCE_ANY
        // reusing regForPrimaryPort for signalling.
        pthread_mutex_lock(&ctrMutex);
        regForPrimaryPort = 1;
        pthread_cond_signal(&ctrCond);
        pthread_mutex_unlock(&ctrMutex);
        LOGI("regForPrimaryPort signalled.");
        break;
    case QMI_CLIENT_SERVICE_COUNT_DEC:
        LOGI("QMI AT service is down.", (int) service_event);
        //no need to release again. QCCI error_cb should have provided
        //this info already.
        break;
    default:
        LOGI("Recvd unsupported event from QCCI notifier.", (int) service_event);
        break;
    }
}

/*===========================================================================
  FUNCTION  qmi_at_unsol_ind_cb

  New indication callback to be provided with qmi_client_init_instance.
  Note. This function uses helper functions from legacy qcci to process
  raw indication data. Once AT QMI qcci service files are ready, use
  decode function instead.

===========================================================================*/
void qmi_at_unsol_ind_cb
(
  qmi_client_type                user_handle,
  unsigned int                   msg_id,
  void                          *ind_buf,
  unsigned int                   ind_buf_len,
  void                          *ind_cb_data
)
{
    qmi_atcop_indication_data_type ind_data;
    LOGI("qmi_at_unsol_ind_cb: ind id:%d \n", msg_id);

    switch ( msg_id )
    {
    case QMI_AT_ABORT_AT_CMD_IND_V01:
        LOGI("Received abort message from QMI\n"); //nothing to do
        break;
    case QMI_AT_FWD_AT_CMD_IND_V01:
        LOGI("Received AT command forward request from modem\n");
        pthread_mutex_lock(&ctrMutex);
        qmi_atcop_srvc_indication_cb_helper (
            msg_id,
            ind_buf,
            ind_buf_len,
            &ind_data
            );
        commandHandle = ind_data.at_hndl;
        request = ind_data.at_cmd_fwd_type;
        parseInput();
        newRequest = 1;
        pthread_cond_signal(&ctrCond);
        pthread_mutex_unlock(&ctrMutex);
        LOGI("ctrCond signalling complete.\n");
        break;
    }
}

static void atfwdSysEventHandler (qmi_sys_event_type event_id,
        const qmi_sys_event_info_type *event_info, void *user_data) {

    int ret;
    const char* devId;

    if (event_info && event_info->qmi_modem_service_ind.dev_id) {
        devId = event_info->qmi_modem_service_ind.dev_id;
        LOGI("Received sys_event: %d from QMI devId: %s\n", (int) event_id, devId);
        if ((qmiPort && !strcmp(qmiPort, devId))
                || (secondaryPort && !strcmp(secondaryPort, devId))) {
            switch (event_id) {
                case QMI_SYS_EVENT_MODEM_OUT_OF_SERVICE_IND:
                    LOGI("Modem Out Of Service --> Release ATCOP service client handles, if any\n");
                    if (qmiPort && !strcmp(qmiPort, devId) && (userHandle > 0)) {
                        qmiErrorCode = 0;
                        ret = qmi_atcop_srvc_release_client(userHandle, &qmiErrorCode);
                        LOGI("release_client returns %d, qmiErroCode = %d for qmiPort: %s & "
                                "userHandle: %d\n", ret, qmiErrorCode, qmiPort, userHandle);
                        userHandle = -1;
                    }
                    if (secondaryPort && !strcmp(secondaryPort, devId) && (userHandleSMD > 0)) {
                        qmiErrorCode = 0;
                        ret = qmi_atcop_srvc_release_client(userHandleSMD, &qmiErrorCode);
                        LOGI("release_client returns %d, qmiErroCode = %d for secondaryPort: %s & "
                            "userHandleSMD: %d\n", ret, qmiErrorCode, secondaryPort, userHandleSMD);
                        userHandleSMD = -1;
                    }
                    break;

                case QMI_SYS_EVENT_MODEM_IN_SERVICE_IND:
                    LOGI("Modem In Service --> Notify Main loop to init ATCOP service client\n");
                    if (qmiPort && !strcmp(qmiPort, devId)) {
                        pthread_mutex_lock(&ctrMutex);
                        regForPrimaryPort = 1;
                        pthread_cond_signal(&ctrCond);
                        pthread_mutex_unlock(&ctrMutex);
                    }
                    if (secondaryPort && !strcmp(secondaryPort, devId)) {
                        pthread_mutex_lock(&ctrMutex);
                        regForSecondaryPort = 1;
                        pthread_cond_signal(&ctrCond);
                        pthread_mutex_unlock(&ctrMutex);
                    }

                    break;

                default:
                    LOGI("Unhandled SYS event (%d) from QMI....\n", (int) event_id);
                    break;
            }
        }
    } else {
        LOGI("Ignoring sys_event: %d from QMI with NO event_info/dev_id\n", (int) event_id);
    }
}

void tryInit (atfwd_init_type_t type, int *result) {
    int retryCnt = 1;
    qmi_client_error_type client_err = QMI_NO_ERR;
    int num_retries = 0;

    for (; retryCnt <= ATFWD_MAX_RETRY_ATTEMPTS; retryCnt++) {
        LOGI("\n");
        LOGI("\n TrtyInit: retryCnt: %d",retryCnt );
        qmiErrorCode = 0;
        switch (type) {
        case INIT_QMI:
            if (!is_supported_qcci())
            {
                qmiHandle = qmi_init(atfwdSysEventHandler, NULL);
                *result = qmiHandle;
            }
            else
            {
                LOGI("Using QCCI. Skipping qmi_init");
                *result=1; //to support backwards compatibility
            }
            break;
        case INIT_QMI_SRVC:
            if ( !is_supported_qcci())
            {
                *result = qmi_connection_init(qmiPort, &qmiErrorCode);
            }
            else
            {
                 do {
                     if ( num_retries != 0) {
                         sleep(1);
                         LOGI("qmi_client_init_instance status retry : %d", num_retries);
                     }

                     LOGI("\n qmi_client_init_instance....");
                     client_err = qmi_client_init_instance(qmi_at_svc_obj,
                                                   QMI_CLIENT_INSTANCE_ANY,
                                                   qmi_at_unsol_ind_cb,
                                                   NULL,
                                                   &qmi_at_os_params,
                                                   (int) QMI_AT_CLIENT_INIT_TIMEOUT,
                                                   &qmi_at_svc_client);
                     num_retries++;
                 } while ( (client_err != QMI_NO_ERR) && (num_retries < MAX_QMI_AT_SVC_CONNECT_ATTEMPTS) );

                 LOGI("qmi_client_init_instance status: %d, num_retries: %d, retryCnt: %d",
                      (int) client_err, num_retries, retryCnt);

                 // Only qmi_client_init_instance failure will be treated as catastrophic
                 // failure of error_cb and notifier funtions will result in ATFdeamon
                 // not working after SSR.
                 *result = client_err;

                 if ( client_err == QMI_NO_ERR )
                 {
                     client_err = qmi_client_register_error_cb(qmi_at_svc_client,
                                                               at_fwd_sys_down_evt_handler,
                                                               (void*)(intptr_t)0);
                     LOGI("qmi_client_register_error_cb status: %d", (int) client_err);
                 }

                 if ( client_err == QMI_NO_ERR )
                 {
                     client_err =  qmi_client_notifier_init(qmi_at_svc_obj,
                                                            &qmi_at_os_params,
                                                            &qmi_at_notifier
                                                            );
                     LOGI("qmi_client_notifier_init status: %d", (int) client_err);

                     client_err = qmi_client_register_notify_cb(qmi_at_svc_client,
                                                                at_fwd_sys_up_handler,
                                                                NULL);

                     LOGI("qmi_client_notifier_init status: %d", (int) client_err);
                 }
                }//end qcci client init
                break;
            case INIT_ATFWD_SRVC:
                LOGI("type - %d do nothing", type);
                break;
            default:
                LOGI("Invalid type %d", type);
                return;
        }
        LOGI("result : %d \t ,Init step :%d \t ,qmiErrorCode: %d", *result, type, qmiErrorCode);
        if (*result >= 0 && qmiErrorCode == 0) {
            break;
        }
        LOGI("\n Sleeping...");
        sleep(retryCnt * ATFWD_RETRY_DELAY);
        LOGI("\n Out of sleep...");
    }
    LOGI("\n Back to main.");

    return;
}

int isNewCommandAvailable() {
    if (newRequest || regForPrimaryPort || regForSecondaryPort) {
        return 1;
    }
    return 0;
}

void freeAtCmdResponse(AtCmdResponse *response) {
    if (!response) return;
        if (response->response)
            free(response->response);
    free(response);
}

/*=========================================================================
  FUNCTION:  main

===========================================================================*/
/*!
@brief
  Initialize the QMI connection and register the ATFWD event listener.
  argv[1] if provided, gives the name of the qmi port to open.
  Default is "rmnet_sdio0".

*/
/*=========================================================================*/
int main (int argc, char **argv)
{
    AtCmdResponse *response;
    int i, connectionResult, initType;

    userHandle = userHandleSMD = -1;
    i = connectionResult = 0;

    LOGI("*** Starting ATFWD-daemon *** \n");

    if ( !is_supported_qcci() )
    {
        if (argc >= 2) {
            qmiPort = argv[1];
        } else {
            qmiPort = QMI_PORT_RMNET_0;
        }

        if (argc >= 3) {
            secondaryPort = argv[2];
        } else {
            secondaryPort = DEFAULT_SMD_PORT;
        }
    }

    LOGI("init all signals\n");
    signalInit();

    pthread_mutexattr_t attr;
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&ctrMutex, &attr);
    pthread_cond_init(&ctrCond, NULL);

    LOGI("Explicitly disbling qmux \n");
    qmi_cci_qmux_xport_unregister(QMI_CLIENT_QMUX_RMNET_INSTANCE_0);
    qmi_cci_qmux_xport_unregister(QMI_CLIENT_QMUX_RMNET_USB_INSTANCE_0);
    qmi_cci_qmux_xport_unregister(QMI_CLIENT_QMUX_RMNET_SMUX_INSTANCE_0);
    qmi_cci_qmux_xport_unregister(QMI_CLIENT_QMUX_RMNET_MHI_INSTANCE_0);
    LOGI("Disabling QMUX complete...\n");

    //Get QMI service object
    LOGI("getting at svc obj for access terminal QMI svc\n");
    qmi_at_svc_obj = at_get_service_object_v01();

    for (initType = INIT_QMI; initType != INIT_MAX; initType++) {
        connectionResult = 0;
        tryInit (initType, &connectionResult);
        LOGI(" tryinit complete with connectresult: %d\n", connectionResult);
        if (connectionResult < 0)
        {
            if ( !is_supported_qcci() )
            {
              if (qmiHandle >= 0) {
                  qmi_release(qmiHandle);
              }
            }
            else
            {
                LOGI("Release qmi_client...\n");
                qmi_client_release(qmi_at_svc_client);
                qmi_at_svc_client = NULL;
            }

            stopSelf();
            return -1;
        }
    }

    if ( !is_supported_qcci() )
    {
        if (qmiPort) {
            LOGI("ATFWD --> QMI Port : %s\n" , qmiPort);
            initAtcopServiceAndRegisterCommands(qmiPort, &userHandle);
            regForPrimaryPort = 0;
        }

        if (secondaryPort) {
            LOGI("ATFWD --> secondaryPort : %s\n" , secondaryPort);
            initAtcopServiceAndRegisterCommands(secondaryPort, &userHandleSMD);
            regForSecondaryPort = 0;
        }

        if(userHandle < 0 && userHandleSMD < 0) {
            LOGI("Could not register userhandle(s) with both 8k and 9k modems -- bail out");
            if (qmiHandle >= 0) {
                qmi_release(qmiHandle);
            }
            stopSelf();
            return -1;
        }
    }
    else
    {
       if (!registerATCommands())
       {
        stopSelf();
        return -1;
       }
    }

    while (1) {
        pthread_mutex_lock(&ctrMutex);
        while (!isNewCommandAvailable()) {
            LOGI("Waiting for ctrCond");
            pthread_cond_wait(&ctrCond, &ctrMutex);
            LOGI("Recieved ctrCond: p: %d, S:%d, nr: %d",regForPrimaryPort, regForSecondaryPort, newRequest );
        }

        if ( !is_supported_qcci() )
        {
            if (regForPrimaryPort == 1) {
                if (qmiPort) {
                    LOGI("Rcvd pthread notification for primary QMI port registration");
                    initAtcopServiceAndRegisterCommands(qmiPort, &userHandle);
                } else {
                    LOGI("Notification for primary QMI port registration when NOT valid, ignore...");
                }
                regForPrimaryPort = 0;
            }

            if (regForSecondaryPort == 1) {
                if (secondaryPort) {
                    LOGI("Rcvd pthread notification for secondary QMI port registration");
                    initAtcopServiceAndRegisterCommands(secondaryPort, &userHandleSMD);
                } else {
                    LOGI("Notification for secondary QMI port registration when NOT valid, ignore...");
                }
                regForSecondaryPort = 0;
            }
            if(userHandle < 0 && userHandleSMD < 0)
            {
                LOGI("userhandle(s) for both 8k and 9k modems NOT valid -- bail out");
                if (qmiHandle >= 0)
                {
                    qmi_release(qmiHandle);
                }
                stopSelf();
                return -1;
            }
        }
        else
        {
            if ( regForPrimaryPort == 1)
            {
                LOGI("Registering for primary port (QCCI).");
                if (!registerATCommands())
                {
                    LOGI("Register for primary port (QCCI) failed.");
                    stopSelf();
                    return -1;
                }
                regForPrimaryPort=0;
            }
        }

        if (newRequest == 1) {

            response = calloc(1, sizeof(AtCmdResponse));
            if ( response != NULL )
            {
                LOGI("Sending response.");
                sendResponse(response);
                LOGI("Send response complete.");

                if(!strncasecmp(fwdcmd.name, "+CFUN", strlen("+CFUN"))) {
                    system("reboot");
                } else if (!strncasecmp(fwdcmd.name, "$QCPWRDN", strlen("$QCPWRDN"))) {
                    LOGI("QCPWRDN has been detected");
                    /* Halt the UE/APPS */
                    system("halt");
                }

                if (fwdcmd.name) free(fwdcmd.name);
                if (fwdcmd.tokens) {
                    for (i = 0; i < fwdcmd.ntokens; i++) {
                        free(fwdcmd.tokens[i]);
                    }
                    free(fwdcmd.tokens);
                }
                freeAtCmdResponse(response);
            }
            else
            {
                LOGI("Failed to allocate memory for response.");
            }
            newRequest = 0;
            LOGI("New request processing complete.");
        }

        pthread_mutex_unlock(&ctrMutex);
    }

    return 0;
}
