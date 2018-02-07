/*************************************************************************************************************
Copyright (c) 2012-2015, Symphony Teleca Corporation, a Harman International Industries, Incorporated company
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS LISTED "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS LISTED BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
Attributions: The inih library portion of the source code is licensed from 
Brush Technology and Ben Hoyt - Copyright (c) 2009, Brush Technology and Copyright (c) 2009, Ben Hoyt. 
Complete license and copyright information can be found at 
https://github.com/benhoyt/inih/commit/74d2ca064fb293bc60a77b0bd068075b293cf175.
*************************************************************************************************************/

/*
* MODULE SUMMARY : ALSA interface module.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <assert.h>
#include "openavb_types_pub.h"
#include "openavb_audio_pub.h"
#include "openavb_trace_pub.h"
#include "openavb_mediaq_pub.h"
#include "openavb_map_uncmp_audio_pub.h"
#include "openavb_map_aaf_audio_pub.h"
#include "openavb_intf_pub.h"
#include "openavb_reference_clock_pub.h"

#define	AVB_LOG_COMPONENT	"ALSA Interface"
#include "openavb_log_pub.h"

// The asoundlib.h header needs to appear after openavb_trace_pub.h otherwise an incompatibtily version of time.h gets pulled in.
#include <sound/asound.h>
#include <alsa-intf/alsa_audio.h>

#define PCM_DEVICE_NAME_DEFAULT	"default"
#define PCM_ACCESS_TYPE			SND_PCM_ACCESS_RW_INTERLEAVED

#define DEBUG 0
#define DEBUG_MCR 0

#define MUTEX_LOCK_ERR(mutex_handle) { \
	MUTEX_CREATE_ERR(); \
	MUTEX_LOCK(mutex_handle); \
	MUTEX_LOG_ERR("Mutex Lock failure"); \
}

#define MUTEX_UNLOCK_ERR(mutex_handle) { \
	MUTEX_CREATE_ERR(); \
	MUTEX_UNLOCK(mutex_handle); \
	MUTEX_LOG_ERR("Mutex Unlock failure"); \
}

typedef struct {
	/////////////
	// Config data
	/////////////
	// Ignore timestamp at listener.
	bool ignoreTimestamp;

	// ALSA Device name
	char *pDeviceName;

	// map_nv_audio_rate
	avb_audio_rate_t audioRate;

	// map_nv_audio_type
	avb_audio_type_t audioType;

	// map_nv_audio_bit_depth
	avb_audio_bit_depth_t audioBitDepth;

	// map_nv_audio_endian
	avb_audio_endian_t audioEndian;

	// map_nv_channels
	avb_audio_channels_t audioChannels;

	// map_nv_allow_resampling
	bool allowResampling;

	U32 startThresholdPeriods;

	U32 periodTimeUsec;

	char *pCh2Filename;

	U32 sampleSizeBytes;

	U32 frameSizeBytes;

	// This is used by the listener.
	// 1 if the clock tick is to be generated when the audio is consumed.
	// 0 otherwise.
	U16 genClockTickOnConsumeAudio;

	// intf_nv_clock_source_timestamp_interval
	U32 clockSourceTimestampInterval;

	// intf_nv_clock_source_timestamp_throwaway
	// This is used by the listener when acting as a clock source.
	// This specifies how many clock reference timestamps to throwaway
	// when starting the stream to allow the ALSA buffer to fill and
	// the consume rate to stabilize.
	U32 clockSourceTimestampThrowaway;

	// intf_nv_sync_to_clock_tick_on_tx_audio
	// This is used by the talker.
	// 1 if the talker should synchronize to the reference clock when
	// transmitting audio.
	// 0 otherwise.
	U16 syncToClockTickOnTxAudio;

	// intf_nv_clock_recovery_adjustment_range
	// This useused by the talker.
	// Clock recovery adjustment range. If the actual number of frames
	// minus the reference number of frames is off by this amount,
	// then we'll add one frame at the next clock tick.
	U16 clockRecoveryAdjustmentRange;

	/////////////
	// Variable data
	/////////////
	// Handle for the PCM device
	struct pcm *pcmHandle;

	// ALSA read/write interval
	U32 intervalCounter;

	int ch2Fd;

	// VARIABLES FOR TALKER REFERENCE CLOCK

	// The number of frames that have been transmitted minus
	// the number of frames that should have been transmitted according to
	// the reference clock. If this number if negative, then we haven't
	// transmitted enough frames and we need to repeat frames to catch up.
	// If this number is positive, then we transmitted too many frames and
	// we need to remove frames to catch up.
	S32 actualMinusRefFrames;

	// The number of frames that are necessary to maintain the clock
	// synchronization. If this is negative, then we need to repeat this
	// many frames before the next clock tick. If this is positive,
	// then we need to remove this many frames before the
	// next clock tick. This is computed by actualMinusRefFrames
	// divided by the clockRecoveryAdjustmentRange
	S32 framesToRecoverClock;

	// Mutex for actualMinusRefFrames
	MUTEX_HANDLE(mtxActualMinusRefFrames);

	// True if the talker has begun transmitting
	bool txStarted;

	// True if the reference clock has started
	bool refClkStarted;

	// The scratch audio buffer.
	// When frames need to be added or removed for clock synchronization,
	// extra frames are stored in this buffer. This is necessary because
	// audio reads must be in multiples of the period size.
	U8 *audioBuffer;
	U32 audioBufferSize;
	U32 audioBufferStartIdx;

	// VARIABLES FOR LISTENER REFERENCE CLOCK

	// The number of frames that have been passed to ALSA by the listener.
	U32 framesConsumed;

} pvt_data_t;


static int x_AVBAudioFormatToAlsaFormat(avb_audio_type_t type,
											  avb_audio_bit_depth_t bitDepth,
											  avb_audio_endian_t endian,
											  char const* pMediaQDataFormat)
{
	bool tight = FALSE;
	if (bitDepth == AVB_AUDIO_BIT_DEPTH_24BIT) {
		if (pMediaQDataFormat != NULL
			&& (strcmp(pMediaQDataFormat, MapAVTPAudioMediaQDataFormat) == 0
			|| strcmp(pMediaQDataFormat, MapUncmpAudioMediaQDataFormat) == 0)) {
			tight = TRUE;
		}
	}

	if (type == AVB_AUDIO_TYPE_FLOAT) {
		switch (bitDepth) {
			case AVB_AUDIO_BIT_DEPTH_32BIT:
				if (endian == AVB_AUDIO_ENDIAN_BIG) {
					return SNDRV_PCM_FORMAT_FLOAT_BE;
				} else {
					return SNDRV_PCM_FORMAT_FLOAT_LE;
				}
			default:
				AVB_LOGF_ERROR("Unsupported audio bit depth for float: %d", bitDepth);
			break;
		}
	}
	else if (type == AVB_AUDIO_TYPE_UINT) {
		switch (bitDepth) {
			case AVB_AUDIO_BIT_DEPTH_8BIT:
				return SNDRV_PCM_FORMAT_U8;
			case AVB_AUDIO_BIT_DEPTH_16BIT:
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_U16_BE;
					} else {
						return SNDRV_PCM_FORMAT_U16_LE;
					}
			case AVB_AUDIO_BIT_DEPTH_20BIT:
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_U20_3BE;
					} else {
						return SNDRV_PCM_FORMAT_U20_3LE;
					}
			case AVB_AUDIO_BIT_DEPTH_24BIT:
			    if (tight) {
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_U24_3BE;
					} else {
						return SNDRV_PCM_FORMAT_U24_3LE;
					}
				}
				else {
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_U24_BE;
					} else {
						return SNDRV_PCM_FORMAT_U24_LE;
					}
				}
			case AVB_AUDIO_BIT_DEPTH_32BIT:
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_U32_BE;
					} else {
						return SNDRV_PCM_FORMAT_U32_LE;
					}
			case AVB_AUDIO_BIT_DEPTH_1BIT:
			case AVB_AUDIO_BIT_DEPTH_48BIT:
			case AVB_AUDIO_BIT_DEPTH_64BIT:
			default:
				AVB_LOGF_ERROR("Unsupported integer audio bit depth: %d", bitDepth);
				break;
		}
	}
	else {
		// AVB_AUDIO_TYPE_INT
		// or unspecified (defaults to signed int)
		switch (bitDepth) {
			case AVB_AUDIO_BIT_DEPTH_8BIT:
				// 8bit samples don't worry about endianness,
				// but default to unsigned instead of signed.
				if (type == AVB_AUDIO_TYPE_INT) {
					return SNDRV_PCM_FORMAT_S8;
				} else { // default
					return SNDRV_PCM_FORMAT_U8;
				}
			case AVB_AUDIO_BIT_DEPTH_16BIT:
				if (endian == AVB_AUDIO_ENDIAN_BIG) {
					return SNDRV_PCM_FORMAT_S16_BE;
				} else {
					return SNDRV_PCM_FORMAT_S16_LE;
				}
			case AVB_AUDIO_BIT_DEPTH_20BIT:
				if (endian == AVB_AUDIO_ENDIAN_BIG) {
					return SNDRV_PCM_FORMAT_S20_3BE;
				} else {
					return SNDRV_PCM_FORMAT_S20_3LE;
				}
			case AVB_AUDIO_BIT_DEPTH_24BIT:
				if (tight) {
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_S24_3BE;
					} else {
						return SNDRV_PCM_FORMAT_S24_3LE;
					}
				} else {
					if (endian == AVB_AUDIO_ENDIAN_BIG) {
						return SNDRV_PCM_FORMAT_S24_BE;
					} else {
						return SNDRV_PCM_FORMAT_S24_LE;
					}
				}
			case AVB_AUDIO_BIT_DEPTH_32BIT:
				if (endian == AVB_AUDIO_ENDIAN_BIG) {
					return SNDRV_PCM_FORMAT_S32_BE;
				} else {
					return SNDRV_PCM_FORMAT_S32_LE;
				}
			case AVB_AUDIO_BIT_DEPTH_1BIT:
			case AVB_AUDIO_BIT_DEPTH_48BIT:
			case AVB_AUDIO_BIT_DEPTH_64BIT:
			default:
				AVB_LOGF_ERROR("Unsupported audio bit depth: %d", bitDepth);
				break;
		}
	}

	return -1;
}


// Each configuration name value pair for this mapping will result in this callback being called.
void openavbIntfAlsa2CfgCB(media_q_t *pMediaQ, const char *name, const char *value) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	if (pMediaQ) {
		char *pEnd;
		long tmp;
		U32 val;

		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo;
		pPubMapUncmpAudioInfo = (media_q_pub_map_uncmp_audio_info_t *)pMediaQ->pPubMapInfo;
		if (!pPubMapUncmpAudioInfo) {
			AVB_LOG_ERROR("Public map data for audio info not allocated.");
			return;
		}


		if (strcmp(name, "intf_nv_ignore_timestamp") == 0) {
			tmp = strtol(value, &pEnd, 10);
			if (*pEnd == '\0' && tmp == 1) {
				pPvtData->ignoreTimestamp = (tmp == 1);
			}
		}

		else if (strcmp(name, "intf_nv_device_name") == 0) {
			if (pPvtData->pDeviceName) {
				free(pPvtData->pDeviceName);
			}
			pPvtData->pDeviceName = strdup(value);
		}

		else if (strcmp(name, "intf_nv_audio_rate") == 0) {
			val = strtol(value, &pEnd, 10);
			// TODO: Should check for specific values
			if (val >= AVB_AUDIO_RATE_8KHZ && val <= AVB_AUDIO_RATE_192KHZ) {
				pPvtData->audioRate = val;
			}
			else {
				AVB_LOG_ERROR("Invalid audio rate configured for intf_nv_audio_rate.");
				pPvtData->audioRate = AVB_AUDIO_RATE_44_1KHZ;
			}

			// Give the audio parameters to the mapping module.
			if (pMediaQ->pMediaQDataFormat) {
				if (strcmp(pMediaQ->pMediaQDataFormat, MapUncmpAudioMediaQDataFormat) == 0
					|| strcmp(pMediaQ->pMediaQDataFormat, MapAVTPAudioMediaQDataFormat) == 0) {
					pPubMapUncmpAudioInfo->audioRate = pPvtData->audioRate;
				}
				//else if (pMediaQ->pMediaQDataFormat == MapSAFMediaQDataFormat) {
				//}
			}
		}

		else if (strcmp(name, "intf_nv_audio_bit_depth") == 0) {
			val = strtol(value, &pEnd, 10);
			// TODO: Should check for specific values
			if (val >= AVB_AUDIO_BIT_DEPTH_1BIT && val <= AVB_AUDIO_BIT_DEPTH_64BIT) {
				pPvtData->audioBitDepth = val;
			}
			else {
				AVB_LOG_ERROR("Invalid audio type configured for intf_nv_audio_bits.");
				pPvtData->audioBitDepth = AVB_AUDIO_BIT_DEPTH_24BIT;
			}

			// Give the audio parameters to the mapping module.
			if (pMediaQ->pMediaQDataFormat) {
				if (strcmp(pMediaQ->pMediaQDataFormat, MapUncmpAudioMediaQDataFormat) == 0
					|| strcmp(pMediaQ->pMediaQDataFormat, MapAVTPAudioMediaQDataFormat) == 0) {
					pPubMapUncmpAudioInfo->audioBitDepth = pPvtData->audioBitDepth;
				}
				//else if (pMediaQ->pMediaQDataFormat == MapSAFMediaQDataFormat) {
				//}
			}
		}

		else if (strcmp(name, "intf_nv_audio_type") == 0) {
			if (strncasecmp(value, "float", 5) == 0) {
				pPvtData->audioType = AVB_AUDIO_TYPE_FLOAT;
			} else if (strncasecmp(value, "sign", 4) == 0
					 || strncasecmp(value, "int", 4) == 0) {
				pPvtData->audioType = AVB_AUDIO_TYPE_INT;
			} else if (strncasecmp(value, "unsign", 6) == 0
					 || strncasecmp(value, "uint", 4) == 0) {
				pPvtData->audioType = AVB_AUDIO_TYPE_UINT;
			} else {
				AVB_LOG_ERROR("Invalid audio type configured for intf_nv_audio_type.");
				pPvtData->audioType = AVB_AUDIO_TYPE_UNSPEC;
			}

			// Give the audio parameters to the mapping module.
			if (pMediaQ->pMediaQDataFormat) {
				if (strcmp(pMediaQ->pMediaQDataFormat, MapUncmpAudioMediaQDataFormat) == 0
					|| strcmp(pMediaQ->pMediaQDataFormat, MapAVTPAudioMediaQDataFormat) == 0) {
					pPubMapUncmpAudioInfo->audioType = pPvtData->audioType;
				}
				//else if (pMediaQ->pMediaQDataFormat == MapSAFMediaQDataFormat) {
				//}
			}
		}

		else if (strcmp(name, "intf_nv_audio_endian") == 0) {
			if (strncasecmp(value, "big", 3) == 0) {
				pPvtData->audioEndian = AVB_AUDIO_ENDIAN_BIG;
			} else if (strncasecmp(value, "little", 6) == 0) {
				pPvtData->audioEndian = AVB_AUDIO_ENDIAN_LITTLE;
			} else {
				AVB_LOG_ERROR("Invalid audio type configured for intf_nv_audio_endian.");
				pPvtData->audioEndian = AVB_AUDIO_ENDIAN_UNSPEC;
			}

			// Give the audio parameters to the mapping module.
			if (pMediaQ->pMediaQDataFormat) {
				if (strcmp(pMediaQ->pMediaQDataFormat, MapUncmpAudioMediaQDataFormat) == 0
					|| strcmp(pMediaQ->pMediaQDataFormat, MapAVTPAudioMediaQDataFormat) == 0) {
					pPubMapUncmpAudioInfo->audioEndian = pPvtData->audioEndian;
				}
				//else if (pMediaQ->pMediaQDataFormat == MapSAFMediaQDataFormat) {
				//}
			}
		}

		else if (strcmp(name, "intf_nv_audio_channels") == 0) {
			val = strtol(value, &pEnd, 10);
			// TODO: Should check for specific values
			if (val >= AVB_AUDIO_CHANNELS_1 && val <= AVB_AUDIO_CHANNELS_8) {
				pPvtData->audioChannels = val;
			}
			else {
				AVB_LOG_ERROR("Invalid audio channels configured for intf_nv_audio_channels.");
				pPvtData->audioChannels = AVB_AUDIO_CHANNELS_2;
			}

			// Give the audio parameters to the mapping module.
			if (pMediaQ->pMediaQDataFormat) {
				if (strcmp(pMediaQ->pMediaQDataFormat, MapUncmpAudioMediaQDataFormat) == 0
					|| strcmp(pMediaQ->pMediaQDataFormat, MapAVTPAudioMediaQDataFormat) == 0) {
					pPubMapUncmpAudioInfo->audioChannels = pPvtData->audioChannels;
				}
				//else if (pMediaQ->pMediaQDataFormat == MapSAFMediaQDataFormat) {
				//}
			}

		}

		if (strcmp(name, "intf_nv_allow_resampling") == 0) {
			tmp = strtol(value, &pEnd, 10);
			if (*pEnd == '\0' && tmp == 1) {
				pPvtData->allowResampling = (tmp == 1);
			}
		}

		else if (strcmp(name, "intf_nv_start_threshold_periods") == 0) {
			pPvtData->startThresholdPeriods = strtol(value, &pEnd, 10);
		}

		else if (strcmp(name, "intf_nv_period_time") == 0) {
			pPvtData->periodTimeUsec = strtol(value, &pEnd, 10);
		}

		else if (strcmp(name, "intf_nv_ch2_filename") == 0) {
			if (pPvtData->pCh2Filename) {
				free(pPvtData->pCh2Filename);
			}
			pPvtData->pCh2Filename = strdup(value);
		}

		else if (strcmp(name, "intf_nv_clock_source_timestamp_interval") ==
			 0) {
			pPvtData->clockSourceTimestampInterval = strtol(value,
				&pEnd, 10);
			pPubMapUncmpAudioInfo->timestampInterval =
				pPvtData->clockSourceTimestampInterval;
		}

		else if (strcmp(name, "intf_nv_clock_source_timestamp_throwaway") ==
			 0) {
			pPvtData->clockSourceTimestampThrowaway = strtol(value,
				&pEnd, 10);
		}

		else if (strcmp(name, "intf_nv_gen_clock_tick_on_consume_audio") ==
			 0) {
			pPvtData->genClockTickOnConsumeAudio = strtol(value,
				&pEnd, 10);
		}

		else if (strcmp(name, "intf_nv_sync_to_clock_tick_on_tx_audio") ==
			 0) {
			pPvtData->syncToClockTickOnTxAudio = strtol(value,
				&pEnd, 10);
		}

		else if (strcmp(name, "intf_nv_clock_recovery_adjustment_range") ==
			 0) {
			pPvtData->clockRecoveryAdjustmentRange = strtol(value,
				&pEnd, 10);
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfAlsa2GenInitCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo;
	pPubMapUncmpAudioInfo = (media_q_pub_map_uncmp_audio_info_t *)pMediaQ->pPubMapInfo;
	if (!pPubMapUncmpAudioInfo) {
		AVB_LOG_ERROR("Public map data for audio info not allocated.");
		return;
	}

	pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
	if (!pPvtData) {
		AVB_LOG_ERROR("Private interface module data not allocated.");
		return;
	}

	pPvtData->frameSizeBytes = pPubMapUncmpAudioInfo->itemFrameSizeBytes;
	pPvtData->sampleSizeBytes = pPubMapUncmpAudioInfo->itemSampleSizeBytes;

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

static int tx_set_params(struct pcm *pcm)
{
	struct snd_pcm_hw_params *params;
	struct snd_pcm_sw_params *sparams;

	params = (struct snd_pcm_hw_params*) calloc(1, sizeof(struct snd_pcm_hw_params));
	if (!params) {
		AVB_LOG_ERROR("Arec:Failed to allocate ALSA hardware parameters!");
		return -ENOMEM;
	}

	param_init(params);

	param_set_mask(params, SNDRV_PCM_HW_PARAM_ACCESS,
		(pcm->flags & PCM_MMAP)? SNDRV_PCM_ACCESS_MMAP_INTERLEAVED : SNDRV_PCM_ACCESS_RW_INTERLEAVED);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT, pcm->format);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT,
		SNDRV_PCM_SUBFORMAT_STD);
	param_set_min(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, pcm->period_size);
	param_set_int(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 16);
	param_set_int(params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
		pcm->channels * 16);
	param_set_int(params, SNDRV_PCM_HW_PARAM_CHANNELS,
		pcm->channels);
	param_set_int(params, SNDRV_PCM_HW_PARAM_RATE, pcm->rate);
	param_set_hw_refine(pcm, params);
	if (param_set_hw_params(pcm, params)) {
		AVB_LOGF_ERROR("Cannot set hw params: %d", errno);
		return -errno;
	}
	if (DEBUG) {
		param_dump(params);
	}

	pcm->buffer_size = pcm_buffer_size(params);
	pcm->period_size = pcm_period_size(params);
	pcm->period_cnt = pcm->buffer_size/pcm->period_size;
	if (DEBUG) {
		AVB_LOGF_ERROR("period_size (%d)", pcm->period_size);
		AVB_LOGF_ERROR(" buffer_size (%d)", pcm->buffer_size);
		AVB_LOGF_ERROR(" period_cnt  (%d)", pcm->period_cnt);
	}

	sparams = (struct snd_pcm_sw_params*) calloc(1, sizeof(struct snd_pcm_sw_params));
	if (!sparams) {
		AVB_LOG_ERROR("Failed to allocate ALSA software parameters!");
		return -ENOMEM;
	}
	sparams->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
	sparams->period_step = 1;

	if (pcm->flags & PCM_MONO) {
		sparams->avail_min = pcm->period_size/2;
		sparams->xfer_align = pcm->period_size/2;
	} else if (pcm->flags & PCM_QUAD) {
		sparams->avail_min = pcm->period_size/8;
		sparams->xfer_align = pcm->period_size/8;
	} else if (pcm->flags & PCM_5POINT1) {
		sparams->avail_min = pcm->period_size/12;
		sparams->xfer_align = pcm->period_size/12;
	} else {
		sparams->avail_min = pcm->period_size/4;
		sparams->xfer_align = pcm->period_size/4;
	}

	sparams->start_threshold = 1;
	sparams->stop_threshold = INT_MAX;
	sparams->silence_size = 0;
	sparams->silence_threshold = 0;

	if (param_set_sw_params(pcm, sparams)) {
		AVB_LOGF_ERROR("cannot set sw params: %d", errno);
		return -errno;
	}
	if (DEBUG) {
		AVB_LOGF_ERROR("avail_min (%lu)", sparams->avail_min);
		AVB_LOGF_ERROR("start_threshold (%lu)", sparams->start_threshold);
		AVB_LOGF_ERROR("stop_threshold (%lu)", sparams->stop_threshold);
		AVB_LOGF_ERROR("xfer_align (%lu)", sparams->xfer_align);
	}

	return 0;

}

// This is called every time the clock ticks for the talker.
// The interval is the number of frames have should have been transmitted
// since the last clock tick.
static void handleClockTick(void *context, U64 timestamp,
	U16 ticks, U32 interval, bool restartClock)
{
	pvt_data_t *pPvtData = context;

	MUTEX_LOCK_ERR(pPvtData->mtxActualMinusRefFrames);

	// If restartClock is true, then set the counters to 0
	if (restartClock) {
#if DEBUG_MCR
		AVB_LOG_ERROR("Restarting the clock");
#endif
		pPvtData->actualMinusRefFrames = 0;
		pPvtData->framesToRecoverClock = 0;
	}

	if (ticks) {
		pPvtData->refClkStarted = TRUE;
	}

	if (ticks && pPvtData->txStarted) {
		// We received a clock tick so adjust the count by the timing
		// interval.
		pPvtData->actualMinusRefFrames -= ticks * interval;

		// If we haven't transmitted enough frames or we have
		// transmitted too few frames, compute how many frames we
		// need to add or remove to maintain synchronization.
		// This is computed here so that if if the clock stops
		// ticking, the talker will freerun.
		pPvtData->framesToRecoverClock +=
			pPvtData->actualMinusRefFrames /
			pPvtData->clockRecoveryAdjustmentRange;
#if DEBUG_MCR
		if (pPvtData->framesToRecoverClock != 0) {
			AVB_LOGF_ERROR("actualMinusRefFrames: %d, framesToRecoverClock: %d",
				       pPvtData->actualMinusRefFrames,
				       pPvtData->framesToRecoverClock);
		}
#endif
	}

	MUTEX_UNLOCK_ERR(pPvtData->mtxActualMinusRefFrames);
}

// A call to this callback indicates that this interface module will be
// a talker. Any talker initialization can be done in this function.
void openavbIntfAlsa2TxInitCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		// Register the handler for the reference clock ticks
		if (pPvtData->syncToClockTickOnTxAudio) {
			refClkRegisterObserver(handleClockTick, pPvtData);
		}

		// Set the PCM flags to capture audio
		unsigned flags = PCM_IN;
		if (pPvtData->audioChannels == 1) {
			flags |= PCM_MONO;
		} else if (pPvtData->audioChannels == 4) {
			flags |= PCM_QUAD;
		} else if (pPvtData->audioChannels == 6) {
			flags |= PCM_5POINT1;
		} else {
			flags |= PCM_STEREO;
		}

		if (DEBUG) {
			flags |= DEBUG_ON;
		} else {
			flags |= DEBUG_OFF;
		}

		// Open the pcm device.
		pPvtData->pcmHandle = pcm_open(flags, pPvtData->pDeviceName);
		if (!pcm_ready(pPvtData->pcmHandle)) {
			pcm_close(pPvtData->pcmHandle);
			AVB_LOGF_ERROR("pcm_open error: %s", pcm_error(pPvtData->pcmHandle));
			AVB_TRACE_EXIT(AVB_TRACE_INTF);
			return;
		}

		pPvtData->pcmHandle->flags = flags;
		pPvtData->pcmHandle->channels = pPvtData->audioChannels;
		pPvtData->pcmHandle->rate = pPvtData->audioRate;
		pPvtData->pcmHandle->format = x_AVBAudioFormatToAlsaFormat(pPvtData->audioType,
							   pPvtData->audioBitDepth,
							   pPvtData->audioEndian,
							   pMediaQ->pMediaQDataFormat);
		media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo = pMediaQ->pPubMapInfo;
		pPvtData->pcmHandle->period_size = pPubMapUncmpAudioInfo->itemSize;

		tx_set_params(pPvtData->pcmHandle);

		// Get ready for playback
		if (pcm_prepare(pPvtData->pcmHandle)) {
			AVB_LOGF_ERROR("Failed in pcm_prepare: %d", errno);
			pcm_close(pPvtData->pcmHandle);
			return;
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

// read correct amount
static bool readAudioFromAlsa(pvt_data_t *pPvtData, void *buffer, U32 buflen)
{
	// Set the tx_start flag to TRUE so that we can start counting clock
	// ticks.
	pPvtData->txStarted = TRUE;

	// If the reference clock hasn't started, then simply read from
	// ALSA and copy it to the buffer. No nead to do clock sync magic.
	if (!pPvtData->refClkStarted) {
		S32 rslt = pcm_read(pPvtData->pcmHandle, buffer, buflen);
		if (rslt != 0) {
			AVB_LOGF_ERROR("pcm_read() error: %d, %s", rslt, strerror(rslt));
			return FALSE;
		}
		return TRUE;
	}

	// The clock sync magic starts here.

	U32 bufIdx = 0;

	// If the scratch audio buffer hasn't been allocated yet, allocate the audio
	// buffer. Initialize the start index to the end to indicate that there
	// is no data in the buffer.
	if (pPvtData->audioBuffer == NULL) {
		pPvtData->audioBuffer = malloc(buflen);
		if (pPvtData->audioBuffer == NULL) {
			AVB_LOG_ERROR("Failed to allocated memory for audio buffer");
			return FALSE;
		}
		pPvtData->audioBufferSize = buflen;
		// The data is always at the end of the buffer so
		// an empty buffer has audioBufferStartIdx as the same
		// as the audioBufferSize.
		pPvtData->audioBufferStartIdx = pPvtData->audioBufferSize;
	}

	assert(pPvtData->audioBufferSize == buflen);

	// Copy the bytes left in the scratch audio buffer to the output buffer.
	if (pPvtData->audioBufferSize - pPvtData->audioBufferStartIdx > 0) {
		memcpy(buffer,
		       pPvtData->audioBuffer + pPvtData->audioBufferStartIdx,
		       pPvtData->audioBufferSize - pPvtData->audioBufferStartIdx);
		bufIdx += pPvtData->audioBufferSize - pPvtData->audioBufferStartIdx;
		pPvtData->audioBufferStartIdx = pPvtData->audioBufferSize;
	}

	// If we've already filled up the output buffer, return it.
	if (bufIdx == buflen) {
#if DEBUG_MCR
		AVB_LOG_ERROR("extra packet for clock sync.");
#endif
		return TRUE;
	}

	// Now the scratch buffer is empty. Read new data into the scratch buffer.
	S32 rslt = pcm_read(pPvtData->pcmHandle, pPvtData->audioBuffer,
			    pPvtData->audioBufferSize);
	pPvtData->audioBufferStartIdx = 0;

	if (rslt != 0) {
		AVB_LOGF_ERROR("pcm_read() error: %d, %s", rslt, strerror(rslt));
		return FALSE;
	}

	// Fill up the rest of the output buffer with data from the scratch buffer.
	memcpy(buffer + bufIdx, pPvtData->audioBuffer, buflen - bufIdx);
	pPvtData->audioBufferStartIdx += buflen - bufIdx;
	bufIdx = buflen;

	// At this point, the scratch buffer can be empty, or partially full,
	// but it can't be completely full.
	MUTEX_LOCK_ERR(pPvtData->mtxActualMinusRefFrames);

	// Subtract the number of frames read from ALSA. (We're counting the
	// frames as they come out of ALSA and then adjusting by the number
	// of added or removed frames rather than when counting when the frames
	// are queued for transmit because frames come out of ALSA at
	// more regular intervals.)
	pPvtData->actualMinusRefFrames += pPvtData->audioBufferSize /
		pPvtData->frameSizeBytes;
	// Determine how many frames to remove or repeat.
	// Do this inside the critical section but do the actual remove or
	// repeating of frames outside the critical section.
	S32 frames_repeated = 0;
	S32 frames_removed = 0;

	// If the number of frames to recover clock is negative, then we need to
	// repeat this many frames to catch up. But we can only repeat as
	// many frames as we have space in the scratch buffer. We'll make up
	// the remainder the next time this function is called.
	if (pPvtData->framesToRecoverClock < 0) {
		frames_repeated = -1 * pPvtData->framesToRecoverClock;
		if ((pPvtData->audioBufferStartIdx / pPvtData->frameSizeBytes) <
		    frames_repeated) {
			frames_repeated = pPvtData->audioBufferStartIdx /
				pPvtData->frameSizeBytes;
		}
	}
	// If we need to remove frames, we can remove as many frames as we want.
	// We'll just have to read from ALSA again if we need to remove more
	// frames than we already have.
	else if (pPvtData->framesToRecoverClock > 0) {
		frames_removed = pPvtData->framesToRecoverClock;
	}

	// Adjust the count actualMinusRefFrames by the number of frames
	// removed or repeated.
	pPvtData->actualMinusRefFrames += frames_repeated - frames_removed;
	pPvtData->framesToRecoverClock += frames_repeated - frames_removed;

	MUTEX_UNLOCK_ERR(pPvtData->mtxActualMinusRefFrames);

	// Repeat the frames as necessary by copying the last frames in the
	// output buffer to the begining of the scratch buffer
	U32 i;
	for (i = 0; i < frames_repeated; i++) {
		pPvtData->audioBufferStartIdx -= pPvtData->frameSizeBytes;
		memcpy(pPvtData->audioBuffer + pPvtData->audioBufferStartIdx,
		       buffer + buflen - pPvtData->frameSizeBytes,
		       pPvtData->frameSizeBytes);
	}

	// Remove the frames as necessary
	for (i = 0; i < frames_removed; i++) {
		// If there are no frames to remove, read some more frames
		if (pPvtData->audioBufferStartIdx >= pPvtData->audioBufferSize) {
#if DEBUG_MCR
			AVB_LOG_ERROR("extra read for clock sync.");
#endif
			rslt = pcm_read(pPvtData->pcmHandle, pPvtData->audioBuffer,
					pPvtData->audioBufferSize);
			pPvtData->audioBufferStartIdx = 0;

			if (rslt != 0) {
				AVB_LOGF_ERROR("pcm_read() error: %d, %s", rslt, strerror(rslt));
				// Return TRUE because we have already filled
				// the output buffer.
				return TRUE;
			}

			// Adjust the count because we just read some more frames
			// from ALSA
			MUTEX_LOCK_ERR(pPvtData->mtxActualMinusRefFrames);
			pPvtData->actualMinusRefFrames += pPvtData->audioBufferSize /
				pPvtData->frameSizeBytes;
			MUTEX_UNLOCK_ERR(pPvtData->mtxActualMinusRefFrames);
		}

		pPvtData->audioBufferStartIdx += pPvtData->frameSizeBytes;
	}

	return TRUE;
}

// This callback will be called for each AVB transmit interval. 
bool openavbIntfAlsa2TxCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF_DETAIL);

	if (pMediaQ) {
		media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo = pMediaQ->pPubMapInfo;
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		media_q_item_t *pMediaQItem = NULL;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return FALSE;
		}
		//put current wall time into tail item used by AAF mapping module
		if ((pPubMapUncmpAudioInfo->sparseMode != TS_SPARSE_MODE_UNSPEC)) {
			pMediaQItem = openavbMediaQTailLock(pMediaQ, TRUE);
			if ((pMediaQItem) && (pPvtData->intervalCounter % pPubMapUncmpAudioInfo->sparseMode == 0)) {
				openavbAvtpTimeSetToWallTime(pMediaQItem->pAvtpTime);
			}
			openavbMediaQTailUnlock(pMediaQ);
			pMediaQItem = NULL;
		}

		if (pPvtData->intervalCounter++ % pPubMapUncmpAudioInfo->packingFactor != 0) {
			return TRUE;
		}

		pMediaQItem = openavbMediaQHeadLock(pMediaQ);
		if (pMediaQItem) {
			if (pMediaQItem->itemSize < pPubMapUncmpAudioInfo->itemSize) {
				AVB_LOG_ERROR("Media queue item not large enough for samples");
			}

			bool rslt = readAudioFromAlsa(pPvtData,
				pMediaQItem->pPubData + pMediaQItem->dataLen,
				pMediaQItem->itemSize - pMediaQItem->dataLen);

			if (!rslt) {
				openavbMediaQHeadUnlock(pMediaQ);
				AVB_TRACE_EXIT(AVB_TRACE_INTF);
				return FALSE;
			}

			pMediaQItem->dataLen = pMediaQItem->itemSize;
			openavbAvtpTimeSetToWallTime(pMediaQItem->pAvtpTime);

			openavbMediaQHeadPush(pMediaQ);

			AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
			return TRUE;
		}
		else {
			AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
			return FALSE;	// Media queue full
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
	return FALSE;
}

static int rx_set_params(struct pcm *pcm)
{
	struct snd_pcm_hw_params *params;
	struct snd_pcm_sw_params *sparams;

	int channels;
	if(pcm->flags & PCM_MONO) {
		channels = 1;
	} else if(pcm->flags & PCM_QUAD) {
		channels = 4;
	} else if(pcm->flags & PCM_5POINT1) {
		channels = 6;
	} else if(pcm->flags & PCM_7POINT1) {
		channels = 8;
	} else {
		channels = 2;
	}

	params = (struct snd_pcm_hw_params*) calloc(1, sizeof(struct snd_pcm_hw_params));
	if (!params) {
		AVB_LOG_ERROR("Failed to allocate ALSA hardware parameters!");
		return -ENOMEM;
	}

	param_init(params);

	param_set_mask(params, SNDRV_PCM_HW_PARAM_ACCESS,
		(pcm->flags & PCM_MMAP)? SNDRV_PCM_ACCESS_MMAP_INTERLEAVED : SNDRV_PCM_ACCESS_RW_INTERLEAVED);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT, pcm->format);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_SUBFORMAT,
		SNDRV_PCM_SUBFORMAT_STD);
	param_set_min(params, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, pcm->period_size);
	param_set_int(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS, 16);
	param_set_int(params, SNDRV_PCM_HW_PARAM_FRAME_BITS,
		pcm->channels * 16);
	param_set_int(params, SNDRV_PCM_HW_PARAM_CHANNELS,
		pcm->channels);
	param_set_int(params, SNDRV_PCM_HW_PARAM_RATE, pcm->rate);
	param_set_hw_refine(pcm, params);

	if (param_set_hw_params(pcm, params)) {
		AVB_LOGF_ERROR("Aplay:cannot set hw params: %d", errno);
		return -errno;
	}
	if (DEBUG) {
		param_dump(params);
	}

	pcm->buffer_size = pcm_buffer_size(params);
	pcm->period_size = pcm_period_size(params);
	pcm->period_cnt = pcm->buffer_size/pcm->period_size;
	if (DEBUG) {
		AVB_LOGF_ERROR("period_cnt = %d", pcm->period_cnt);
		AVB_LOGF_ERROR("period_size = %d", pcm->period_size);
		AVB_LOGF_ERROR("buffer_size = %d", pcm->buffer_size);
	}

	sparams = (struct snd_pcm_sw_params*) calloc(1, sizeof(struct snd_pcm_sw_params));
	if (!sparams) {
		AVB_LOG_ERROR("Failed to allocate ALSA software parameters!");
		return -ENOMEM;
	}
	// Get the current software parameters
	sparams->tstamp_mode = SNDRV_PCM_TSTAMP_NONE;
	sparams->period_step = 1;

	sparams->avail_min = pcm->period_size/(channels * 2) ;
	sparams->start_threshold =  pcm->period_size/(channels * 2) ;
	sparams->stop_threshold =  pcm->buffer_size ;
	sparams->xfer_align =  pcm->period_size/(channels * 2) ; /* needed for old kernels */

	sparams->silence_size = 0;
	sparams->silence_threshold = 0;

	if (param_set_sw_params(pcm, sparams)) {
		AVB_LOGF_ERROR("Cannot set sw params: %d", errno);
		return -errno;
	}
	if (DEBUG) {
		AVB_LOGF_ERROR("sparams->avail_min= %lu", sparams->avail_min);
		AVB_LOGF_ERROR(" sparams->start_threshold= %lu", sparams->start_threshold);
		AVB_LOGF_ERROR(" sparams->stop_threshold= %lu", sparams->stop_threshold);
		AVB_LOGF_ERROR(" sparams->xfer_align= %lu", sparams->xfer_align);
		AVB_LOGF_ERROR(" sparams->boundary= %lu", sparams->boundary);
	}

	return 0;
}

// A call to this callback indicates that this interface module will be
// a listener. Any listener initialization can be done in this function.
void openavbIntfAlsa2RxInitCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		// Set the flags to play audio
		unsigned flags = PCM_OUT;
		if (pPvtData->audioChannels == 1) {
			flags |= PCM_MONO;
		} else if (pPvtData->audioChannels == 4) {
			flags |= PCM_QUAD;
		} else if (pPvtData->audioChannels == 6) {
			flags |= PCM_5POINT1;
		} else if (pPvtData->audioChannels == 8) {
			flags |= PCM_7POINT1;
		} else {
			flags |= PCM_STEREO;
		}

		if (DEBUG) {
		    flags |= DEBUG_ON;
		} else {
		    flags |= DEBUG_OFF;
		}

		// Open the pcm device.
		pPvtData->pcmHandle = pcm_open(flags, pPvtData->pDeviceName);
		if (!pcm_ready(pPvtData->pcmHandle)) {
			pcm_close(pPvtData->pcmHandle);
			AVB_LOG_ERROR("pcm_open error");
			AVB_TRACE_EXIT(AVB_TRACE_INTF);
			return;
		}

		pPvtData->pcmHandle->channels = pPvtData->audioChannels;
		pPvtData->pcmHandle->rate = pPvtData->audioRate;
		pPvtData->pcmHandle->flags = flags;
		pPvtData->pcmHandle->format = x_AVBAudioFormatToAlsaFormat(pPvtData->audioType,
									   pPvtData->audioBitDepth,
									   pPvtData->audioEndian,
									   pMediaQ->pMediaQDataFormat);
		media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo = pMediaQ->pPubMapInfo;
		pPvtData->pcmHandle->period_size = pPubMapUncmpAudioInfo->itemSize;

		rx_set_params(pPvtData->pcmHandle);

		// Get ready for playback
		if (pcm_prepare(pPvtData->pcmHandle)) {
			AVB_LOGF_ERROR("pcm_prepare() error: %s", pcm_error(pPvtData->pcmHandle));
			pcm_close(pPvtData->pcmHandle);
			pPvtData->pcmHandle = NULL;
			AVB_TRACE_EXIT(AVB_TRACE_INTF);
			return;
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

static void consumeAudio(pvt_data_t *pPvtData, void *data, U32 dataLen)
{
	S32 rslt;

	rslt = pcm_write(pPvtData->pcmHandle, data, dataLen);
	if (rslt != 0) {
		AVB_LOGF_ERROR("pcm_write: %s", pcm_error(pPvtData->pcmHandle));
		return;
	}

	// Trigger a clock tick if we've consumed enough frames.
	if (pPvtData->genClockTickOnConsumeAudio &&
	    pPvtData->clockSourceTimestampInterval) {
		pPvtData->framesConsumed += dataLen /
			pPvtData->frameSizeBytes;
		while (pPvtData->framesConsumed >= pPvtData->clockSourceTimestampInterval) {
			if (pPvtData->clockSourceTimestampThrowaway <= 0) {
				U64 walltime;
				if (CLOCK_GETTIME64(OPENAVB_CLOCK_WALLTIME, &walltime)) {
					refClkSignalTick(walltime, 1,
						pPvtData->clockSourceTimestampInterval, FALSE);
				}
			}
			else {
				pPvtData->clockSourceTimestampThrowaway--;
			}
			pPvtData->framesConsumed -= pPvtData->clockSourceTimestampInterval;
		}
	}
}

// This callback is called when acting as a listener.
bool openavbIntfAlsa2RxCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF_DETAIL);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;

		bool moreItems = TRUE;

		while (moreItems) {
			media_q_item_t *pMediaQItem = openavbMediaQTailLock(pMediaQ, pPvtData->ignoreTimestamp);
			if (pMediaQItem) {
				if (pMediaQItem->dataLen) {
					consumeAudio(pPvtData,
						pMediaQItem->pPubData,
						pMediaQItem->dataLen);
				}
				openavbMediaQTailPull(pMediaQ);
			} else {
				moreItems = FALSE;
			}
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
	return TRUE;
}

// This callback will be called when the interface needs to be closed. All shutdown should 
// occur in this function.
void openavbIntfAlsa2EndCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		refClkUnregisterObserver(pPvtData);

		if (pPvtData->pcmHandle) {
			pcm_close(pPvtData->pcmHandle);
			pPvtData->pcmHandle = NULL;
		}

		if (pPvtData->ch2Fd > 0) {
			close(pPvtData->ch2Fd);
		}

		if (pPvtData->audioBuffer) {
			free(pPvtData->audioBuffer);
			pPvtData->audioBuffer = NULL;
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

void openavbIntfAlsa2GenEndCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

bool openavbIntfAlsa2TxBlockingInIntfCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);
	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}

// Main initialization entry point into the interface module
extern DLL_EXPORT bool openavbIntfAlsa2Initialize(media_q_t *pMediaQ, openavb_intf_cb_t *pIntfCB)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pMediaQ->pPvtIntfInfo = calloc(1, sizeof(pvt_data_t));		// Memory freed by the media queue when the media queue is destroyed.

		if (!pMediaQ->pPvtIntfInfo) {
			AVB_LOG_ERROR("Unable to allocate memory for AVTP interface module.");
			return FALSE;
		}

		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;

		pIntfCB->intf_cfg_cb = openavbIntfAlsa2CfgCB;
		pIntfCB->intf_gen_init_cb = openavbIntfAlsa2GenInitCB;
		pIntfCB->intf_tx_init_cb = openavbIntfAlsa2TxInitCB;
		pIntfCB->intf_tx_cb = openavbIntfAlsa2TxCB;
		pIntfCB->intf_rx_init_cb = openavbIntfAlsa2RxInitCB;
		pIntfCB->intf_rx_cb = openavbIntfAlsa2RxCB;
		pIntfCB->intf_end_cb = openavbIntfAlsa2EndCB;
		pIntfCB->intf_gen_end_cb = openavbIntfAlsa2GenEndCB;
		pIntfCB->intf_tx_blocking_in_intf_cb =
			openavbIntfAlsa2TxBlockingInIntfCB;

		pPvtData->ignoreTimestamp = FALSE;
		pPvtData->pDeviceName = strdup(PCM_DEVICE_NAME_DEFAULT);
		pPvtData->allowResampling = TRUE;
		pPvtData->intervalCounter = 0;
		pPvtData->startThresholdPeriods = 2;	// Default to 2 periods of frames as the start threshold
		pPvtData->periodTimeUsec = 100000;

		pPvtData->ch2Fd = 0;
		pPvtData->clockSourceTimestampThrowaway = 10;

		pPvtData->clockRecoveryAdjustmentRange = 500;

		MUTEX_ATTR_HANDLE(mta);
		MUTEX_ATTR_INIT(mta);
		MUTEX_ATTR_SET_TYPE(mta, MUTEX_ATTR_TYPE_DEFAULT);
		MUTEX_ATTR_SET_NAME(mta, "mtxActualMinusRefFrames");
		MUTEX_CREATE_ERR();
		MUTEX_CREATE(pPvtData->mtxActualMinusRefFrames, mta);
		MUTEX_LOG_ERR("Error creating mutex");
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}

// A call to this callback indicates that this interface module will be
// a talker. Any talker initialization can be done in this function.
void openavbIntfAlsa2DualTxInitCB(media_q_t *pMediaQ) 
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return;
		}

		// Set the PCM flags to capture audio
		unsigned flags = PCM_IN | PCM_MONO;

		if (DEBUG) {
			flags |= DEBUG_ON;
		} else {
			flags |= DEBUG_OFF;
		}

		// Open the pcm device.
		pPvtData->pcmHandle = pcm_open(flags, pPvtData->pDeviceName);
		if (!pcm_ready(pPvtData->pcmHandle)) {
			pcm_close(pPvtData->pcmHandle);
			AVB_LOGF_ERROR("pcm_open error: %s", pcm_error(pPvtData->pcmHandle));
			AVB_TRACE_EXIT(AVB_TRACE_INTF);
			return;
		}

		pPvtData->pcmHandle->flags = flags;
		pPvtData->pcmHandle->channels = 1;
		pPvtData->pcmHandle->rate = pPvtData->audioRate;
		pPvtData->pcmHandle->format = x_AVBAudioFormatToAlsaFormat(pPvtData->audioType,
							   pPvtData->audioBitDepth,
							   pPvtData->audioEndian,
							   pMediaQ->pMediaQDataFormat);
		media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo = pMediaQ->pPubMapInfo;
		pPvtData->pcmHandle->period_size = pPubMapUncmpAudioInfo->itemSize / pPvtData->audioChannels;

		tx_set_params(pPvtData->pcmHandle);


		// Get ready for playback
		if (pcm_prepare(pPvtData->pcmHandle)) {
			AVB_LOG_ERROR("Failed in pcm_prepare");
			pcm_close(pPvtData->pcmHandle);
			AVB_TRACE_EXIT(AVB_TRACE_INTF);
			return;
		}

		// Open the file for the second channel
		if (pPvtData->pCh2Filename) {
			pPvtData->ch2Fd = open(pPvtData->pCh2Filename, O_RDONLY);
			if (pPvtData->ch2Fd <= 0) {
				AVB_LOGF_ERROR("Cannot open file: %s", pPvtData->pCh2Filename);
				pPvtData->ch2Fd = 0;
			}
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
}

#define SAMPLE_SIZE 2
static bool interleave_samples(uint8_t *destination, unsigned int dest_size,
				 uint8_t *ch1_data, unsigned int ch1_data_size,
				 uint8_t *ch2_data, unsigned int ch2_data_size)
{
	uint8_t *cursor;
	unsigned int max_ch_data_size = (ch1_data_size > ch2_data_size) ?
		ch1_data_size : ch2_data_size;

	if (destination == NULL || (ch1_data_size && ch1_data == NULL) ||
	    (ch2_data_size && ch2_data == NULL)) {
		AVB_LOG_ERROR("Invalid parameter");
		return false;
	}

	/* The channel data must be a multiple of the sample size. */
	if (ch1_data_size % SAMPLE_SIZE != 0 ||
		ch2_data_size % SAMPLE_SIZE != 0) {
		AVB_LOG_ERROR("Invalid frame size");
		return false;
	}

	/* Create the scratch buffer if it's too small or not created yet. */
	if (dest_size < max_ch_data_size * 2) {
		AVB_LOG_ERROR("Buffer is too small");
		return false;
	}

	/* Zero the buffer. */
	memset(destination, 0, max_ch_data_size * 2);

	/* Interleave the samples from channels 1 and 2 into the scratch buffer. */
	cursor = destination;
	while (ch1_data_size || ch2_data_size) {
		if (ch1_data_size) {
			memcpy(cursor, ch1_data, SAMPLE_SIZE);
			ch1_data += SAMPLE_SIZE;
			ch1_data_size -= SAMPLE_SIZE;
		}
		cursor += SAMPLE_SIZE;

		if (ch2_data_size) {
			memcpy(cursor, ch2_data, SAMPLE_SIZE);
			ch2_data += SAMPLE_SIZE;
			ch2_data_size -= SAMPLE_SIZE;
		}
		cursor += SAMPLE_SIZE;
	}

	return true;
}

// This callback will be called for each AVB transmit interval. 
bool openavbIntfAlsa2DualTxCB(media_q_t *pMediaQ)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF_DETAIL);

	if (pMediaQ) {
		media_q_pub_map_uncmp_audio_info_t *pPubMapUncmpAudioInfo = pMediaQ->pPubMapInfo;
		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;
		media_q_item_t *pMediaQItem = NULL;
		if (!pPvtData) {
			AVB_LOG_ERROR("Private interface module data not allocated.");
			return FALSE;
		}

		// Register the handler for the reference clock ticks
		if (pPvtData->syncToClockTickOnTxAudio) {
			refClkRegisterObserver(handleClockTick, pPvtData);
		}

		//put current wall time into tail item used by AAF mapping module
		if ((pPubMapUncmpAudioInfo->sparseMode != TS_SPARSE_MODE_UNSPEC)) {
			pMediaQItem = openavbMediaQTailLock(pMediaQ, TRUE);
			if ((pMediaQItem) && (pPvtData->intervalCounter % pPubMapUncmpAudioInfo->sparseMode == 0)) {
				openavbAvtpTimeSetToWallTime(pMediaQItem->pAvtpTime);
			}
			openavbMediaQTailUnlock(pMediaQ);
			pMediaQItem = NULL;
		}

		if (pPvtData->intervalCounter++ % pPubMapUncmpAudioInfo->packingFactor != 0)
			return TRUE;

		pMediaQItem = openavbMediaQHeadLock(pMediaQ);
		if (pMediaQItem) {
			if (pMediaQItem->itemSize < pPubMapUncmpAudioInfo->itemSize) {
				AVB_LOG_ERROR("Media queue item not large enough for samples");
				openavbMediaQHeadUnlock(pMediaQ);
				AVB_TRACE_EXIT(AVB_TRACE_INTF);
				return FALSE;
			}

			static uint8_t *bufferCh1 = NULL;
			static uint8_t *bufferCh2 = NULL;
			if (bufferCh1 == NULL) {
				bufferCh1 = malloc(pPubMapUncmpAudioInfo->itemSize /
						   pPubMapUncmpAudioInfo->audioChannels);
				bufferCh2 = malloc(pPubMapUncmpAudioInfo->itemSize /
						   pPubMapUncmpAudioInfo->audioChannels);
				if (bufferCh1 == NULL || bufferCh2 == NULL) {
					AVB_LOG_ERROR("Could not allocate buffer");
					openavbMediaQHeadUnlock(pMediaQ);
					AVB_TRACE_EXIT(AVB_TRACE_INTF);
					return FALSE;
				}
			}

			/* Read enough bytes from alsa to fill one channel of the stream */
			uint32_t bytesOnCh1 = (pPubMapUncmpAudioInfo->itemSize - 
				      pMediaQItem->dataLen) / 
				     pPubMapUncmpAudioInfo->audioChannels;
			if (pcm_read(pPvtData->pcmHandle, bufferCh1, bytesOnCh1) != 0) {
				AVB_LOGF_ERROR("pcm_read() error: %d", errno);
				openavbMediaQHeadUnlock(pMediaQ);
				AVB_TRACE_EXIT(AVB_TRACE_INTF);
				return FALSE;
			}

			/* Read enough bytes from the file to fill the second channel of the stream */
			uint32_t bytesOnCh2 = 0;
			if (pPvtData->ch2Fd > 0) {
				bytesOnCh2 = read(pPvtData->ch2Fd, bufferCh2, 
					(pPubMapUncmpAudioInfo->itemSize - pMediaQItem->dataLen) / 
					pPubMapUncmpAudioInfo->audioChannels);
			}

			/* Interleave the sample from ch1 and ch2 to make the stream */
			if (!interleave_samples(pMediaQItem->pPubData + pMediaQItem->dataLen, 
					  pPubMapUncmpAudioInfo->itemSize - pMediaQItem->dataLen,
					  bufferCh1, bytesOnCh1,
					  bufferCh2, bytesOnCh2)) {
					AVB_LOG_ERROR("Could not interleave samples");
					openavbMediaQHeadUnlock(pMediaQ);
					AVB_TRACE_EXIT(AVB_TRACE_INTF);
					return FALSE;
			}

			pMediaQItem->dataLen = pPubMapUncmpAudioInfo->itemSize;
			openavbAvtpTimeSetToWallTime(pMediaQItem->pAvtpTime);
			openavbMediaQHeadPush(pMediaQ);

			AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
			return TRUE;
		}
		else {
			AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
			return FALSE;	// Media queue full
		}
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF_DETAIL);
	return FALSE;
}

// Main initialization entry point into the interface module
extern DLL_EXPORT bool openavbIntfAlsa2DualInitialize(media_q_t *pMediaQ, openavb_intf_cb_t *pIntfCB)
{
	AVB_TRACE_ENTRY(AVB_TRACE_INTF);

	if (pMediaQ) {
		pMediaQ->pPvtIntfInfo = calloc(1, sizeof(pvt_data_t));		// Memory freed by the media queue when the media queue is destroyed.

		if (!pMediaQ->pPvtIntfInfo) {
			AVB_LOG_ERROR("Unable to allocate memory for AVTP interface module.");
			return FALSE;
		}

		pvt_data_t *pPvtData = pMediaQ->pPvtIntfInfo;

		pIntfCB->intf_cfg_cb = openavbIntfAlsa2CfgCB;
		pIntfCB->intf_gen_init_cb = openavbIntfAlsa2GenInitCB;
		pIntfCB->intf_tx_init_cb = openavbIntfAlsa2DualTxInitCB;
		pIntfCB->intf_tx_cb = openavbIntfAlsa2DualTxCB;
		pIntfCB->intf_rx_init_cb = openavbIntfAlsa2RxInitCB;
		pIntfCB->intf_rx_cb = openavbIntfAlsa2RxCB;
		pIntfCB->intf_end_cb = openavbIntfAlsa2EndCB;
		pIntfCB->intf_gen_end_cb = openavbIntfAlsa2GenEndCB;
		pIntfCB->intf_tx_blocking_in_intf_cb =
			openavbIntfAlsa2TxBlockingInIntfCB;

		pPvtData->ignoreTimestamp = FALSE;
		pPvtData->pDeviceName = strdup(PCM_DEVICE_NAME_DEFAULT);
		pPvtData->allowResampling = TRUE;
		pPvtData->intervalCounter = 0;
		pPvtData->startThresholdPeriods = 2;	// Default to 2 periods of frames as the start threshold
		pPvtData->periodTimeUsec = 100000;

		pPvtData->ch2Fd = 0;

		pPvtData->clockRecoveryAdjustmentRange = 500;

		MUTEX_ATTR_HANDLE(mta);
		MUTEX_ATTR_INIT(mta);
		MUTEX_ATTR_SET_TYPE(mta, MUTEX_ATTR_TYPE_DEFAULT);
		MUTEX_ATTR_SET_NAME(mta, "mtxActualMinusRefFrames");
		MUTEX_CREATE_ERR();
		MUTEX_CREATE(pPvtData->mtxActualMinusRefFrames, mta);
		MUTEX_LOG_ERR("Error creating mutex");
	}

	AVB_TRACE_EXIT(AVB_TRACE_INTF);
	return TRUE;
}
