/*============================================================================
* Copyright (c) 2017 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
============================================================================*/

#ifndef QAP_DEFS_H
#define QAP_DEFS_H

__BEGIN_DECLS

#define MAX_SUPPORTED_OUTPUTS 4
#define DTS_MAX_CHANNELS (8)

/* FLAG to be set for input buffers from Client */
typedef enum {
    QAP_BUFFER_NO_TSTAMP,       /* Specifies there's no timestamp for the buffer */
    QAP_BUFFER_EOS,             /* Specifies End of Stream buffer */
    QAP_BUFFER_TSTAMP,          /* Specifies that timestamp is set for this input buffer */
    QAP_BUFFER_TSTAMP_CONTINUE, /* Timestamp is to be continued from previous buffer */
} qap_buffer_flags_t;

/* Audio formats */
typedef enum {
    QAP_AUDIO_FORMAT_PCM_16_BIT,
    QAP_AUDIO_FORMAT_PCM_8_24_BIT,
    QAP_AUDIO_FORMAT_PCM_24_BIT_PACKED,
    QAP_AUDIO_FORMAT_PCM_32_BIT,
    QAP_AUDIO_FORMAT_AC3,
    QAP_AUDIO_FORMAT_AC4,
    QAP_AUDIO_FORMAT_EAC3,
    QAP_AUDIO_FORMAT_AAC,
    QAP_AUDIO_FORMAT_AAC_ADTS,
    QAP_AUDIO_FORMAT_MP2,
    QAP_AUDIO_FORMAT_MP3,
    QAP_AUDIO_FORMAT_FLAC,
    QAP_AUDIO_FORMAT_ALAC,
    QAP_AUDIO_FORMAT_APE,
    QAP_AUDIO_FORMAT_DTS,
    QAP_AUDIO_FORMAT_DTS_HD,
} qap_audio_format_t;

/* Types of usecases - Used for graph creation of modules */
typedef enum {
    QAP_SESSION_BROADCAST,
    QAP_SESSION_DECODE_ONLY,
    QAP_SESSION_ENCODE_ONLY,
} qap_session_t;

/* Module FLAGS */
typedef enum {
    QAP_MODULE_FLAG_PRIMARY = 0x1,       /* Specifies Primary Decoder */
    QAP_MODULE_FLAG_SECONDARY = 0x2,     /* Specifies Secondary Decider */
    QAP_MODULE_FLAG_APP_SOUND = 0x4,     /* Specifies Appln sound */
    QAP_MODULE_FLAG_SYSTEM_SOUND = 0x5,  /* Specifies System sound */
} qap_module_flags_t;

/* Module Types */
typedef enum {
    QAP_MODULE_DECODER = 0,       /* Specifies current module is a Decoder */
    QAP_MODULE_ENCODER,           /* Specifies current module is a Encoder */
    QAP_MODULE_MAX,
} qap_module_type_t;

/* Buffer Metadata Types */
typedef enum {
    QAP_BUFFER_SEC_METADATA_PARAMS = 0, /* Specifies secondary metadata for BD usecases*/
    QAP_BUFFER_MAX,
} qap_buffer_metadata_type_t;

/* Input module config
 *   -- Called using QAP_MODULE_CMD_SET_CONFIG */
typedef struct qap_module_config {
    /* Common params */
    qap_audio_format_t format;
    int32_t sample_rate;
    int channels;
    int bit_width;
    bool is_interleaved;
    qap_module_type_t module_type; /* Specify Module Type */
    qap_module_flags_t flags;      /* Flag indicating Module's attributes */

    /* Decoder specific params */
    int32_t config_size;           /* size of module specific params */
    void *config;                  /* Decoder specific config params */
    /*TBD: Define the module specific params for all the modules */
} qap_module_config_t;

/* Session Output module config
 *   -- Called using QAP_SESSION_CMD_SET_OUTPUTS */
typedef struct qap_output_config {
    qap_audio_format_t format;
    int32_t sample_rate;
    int channels;
    int bit_width;
    bool is_interleaved;
    uint32_t id;          /* Unique Output ID */
} qap_output_config_t;

/* Session Output module config
 *   -- Called using QAP_SESSION_CMD_SET_OUTPUTS */
typedef struct qap_session_outputs_config {
    uint32_t num_output;  /* Number of outputs needed from Module */
    qap_output_config_t output_config[MAX_SUPPORTED_OUTPUTS];
} qap_session_outputs_config_t;

typedef struct qap_buffer_common {
    uint32_t offset;           /* Offset to the first data sample in the buffer */
    uint32_t size;             /* Buffer Size */
    void*   data;             /* Buffer  pointer */
    int64_t timestamp;        /* Inband timestamp for input/output buffer */
} qap_buffer_common_t;

typedef struct qap_input_buff_params {
    qap_buffer_flags_t flags; /* Flags indicating the buffer attributes */
} qap_input_buff_params_t;

typedef struct qap_output_buff_params {
    uint32_t output_id;       /* Unique Output ID */
    void*   metadata;         /* Pointer to metadata. */
                              /* First entry in the metadata specifies type of metadata */
                              /* Possible values : qap_buffer_metadata_type_t */
   qap_output_config_t output_config;
} qap_output_buff_params_t;

/* Common structure for both input and output buffers */
typedef struct qap_audio_buffer {
    qap_buffer_common_t common_params;
    union {
        qap_input_buff_params_t  input_buf_params;
        qap_output_buff_params_t output_buf_params;
    } buffer_parms;
} qap_audio_buffer_t;

typedef enum {
/*    QAP_DECODE_SUCCESS,
    QAP_DECODE_ERROR,*/
    QAP_STATUS_ERR = -1,
    QAP_STATUS_OK = 0,
} qap_status_t;

typedef enum {
    QAP_CALLBACK_EVENT_DATA,              /* Event to notify DATA availabilty to Client */
    QAP_CALLBACK_EVENT_OUTPUT_CFG_CHANGE, /* Event to notify client about output config change */
    QAP_CALLBACK_EVENT_EOS,               /* Event to notify Main Stream EOS */
    QAP_CALLBACK_EVENT_MAIN_2_EOS,        /* Event to notify Second Main Stream EOS */
    QAP_CALLBACK_EVENT_EOS_ASSOC,         /* Event to notify EOS of Assoc stream*/
    QAP_CALLBACK_EVENT_ERROR,             /* Event to notify ERROR */
    QAP_CALLBACK_EVENT_SUCCESS,           /* Event to notify SUCCESS */
    QAP_CALLBACK_EVENT_METADATA,          /* Event to notify METADATA availability */
} qap_callback_event_t;

typedef enum {
    QAP_MODULE_CMD_START,                       /* Module start command */
    QAP_MODULE_CMD_PAUSE,                       /* Module Pause command */
    QAP_MODULE_CMD_FLUSH,                       /* Module Flush command */
    QAP_MODULE_CMD_STOP,                        /* Module Stop command */
    QAP_MODULE_CMD_GET_PARAM,                   /* Command to retrieve run-time module params */
    QAP_MODULE_CMD_SET_PARAM,                   /* Command to set run-time module params */
    QAP_MODULE_CMD_SET_KVPAIRS,                 /* Command to set kvpairs to the module */
    QAP_MODULE_CMD_FIRST_PROPRIETARY = 0x10000, /* first proprietary command code */
} qap_module_cmd_t;

typedef enum {
    QAP_SESSION_CMD_GET_CONFIG,                  /* Get current session ouput config */
    QAP_SESSION_CMD_SET_CONFIG,                  /* Set initial session output config */
    QAP_SESSION_CMD_SET_OUTPUTS,                 /* Set run-time output config */
    QAP_SESSION_CMD_GET_PARAM,                   /* Command to retrieve run-time session params */
    QAP_SESSION_CMD_SET_PARAM,                   /* Command to set run-time session params */
    QAP_SESSION_CMD_SET_KVPAIRS,                 /* Command to set kvpairs to the session */
    QAP_SESSION_CMD_FIRST_PROPRIETARY = 0x10000, /* first proprietary command code */
} qap_session_cmd_t;

typedef void* qap_session_handle_t;
typedef void* qap_module_handle_t;
typedef void* qap_lib_handle_t;

__END_DECLS

#endif //QAP_DEFS_H
