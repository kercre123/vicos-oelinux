/*============================================================================
  NSM hacked to support GC1066

  Copyright (c) 2016 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.

============================================================================*/
#include <stdio.h>
#include "sensor_lib.h"
#include <utils/Log.h>
#include <math.h>

#define SENSOR_MODEL_NO_OV8856_F8V05A "ov8856_f8v05a"
#define OV8856_F8V05A_LOAD_CHROMATIX(n) \
  "libchromatix_"SENSOR_MODEL_NO_OV8856_F8V05A"_"#n".so"

static sensor_lib_t sensor_lib_ptr;

#define LOG_TAG "OV8856"

static struct msm_sensor_power_setting ov8856_f8v05a_power_setting[] = {
    {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VIO,
    .config_val = 0,
    .delay = 1,
  },
  {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VANA,
    .config_val = 0,
    .delay = 1,
  },
/*
  {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VDIG,
    .config_val = 0,
    .delay = 1,
  },
  {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VAF,
    .config_val = 0,
    .delay = 5,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_CUSTOM1,
    .config_val = GPIO_OUT_LOW,
    .delay = 1,
  },
  */
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_STANDBY,
    .config_val = GPIO_OUT_HIGH, // LOW,
    .delay = 1,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_STANDBY,
    .config_val = GPIO_OUT_LOW, // HIGH,
    .delay = 5,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_RESET,
    .config_val = GPIO_OUT_LOW,
    .delay = 5,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_RESET,
    .config_val = GPIO_OUT_HIGH,
    .delay = 10,
  },
  {
    .seq_type = SENSOR_CLK,
    .seq_val = SENSOR_CAM_MCLK,
    .config_val = 24000000,
    .delay = 10,
  },
  {
    .seq_type = SENSOR_I2C_MUX,
    .seq_val = 0,
    .config_val = 0,
    .delay = 0,
  },
};

static struct msm_camera_sensor_slave_info sensor_slave_info = {
  /* Camera slot where this camera is mounted */
  .camera_id = CAMERA_0,
  /* sensor slave address */
  .slave_addr = 0x78,
  /* sensor i2c frequency*/
  .i2c_freq_mode = I2C_FAST_MODE,
  /* sensor address type */
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  /* sensor id info*/
  .sensor_id_info = {
    /* sensor id register address */
    .sensor_id_reg_addr = 0xf0,
    /* sensor id */
    .sensor_id = 0x1066, // GC1066/1606 are same part
  },
  /* power up / down setting */
  .power_setting_array = {
    .power_setting = ov8856_f8v05a_power_setting,
    .size = ARRAY_SIZE(ov8856_f8v05a_power_setting),
  },
  //.is_flash_supported = SENSOR_FLASH_SUPPORTED,
};

static struct msm_sensor_init_params sensor_init_params = {
  .modes_supported = CAMERA_MODE_2D_B,
  .position = BACK_CAMERA_B,
  .sensor_mount_angle = SENSOR_MOUNTANGLE_270,
};

static sensor_output_t sensor_output = {
  .output_format = SENSOR_BAYER,
  .connection_mode = SENSOR_MIPI_CSI,
  .raw_output = SENSOR_10_BIT_DIRECT,
};

static struct msm_sensor_output_reg_addr_t output_reg_addr = {
  .x_output = 0xff, // This is what the other GCxxx drivers do
  .y_output = 0xff, 
  .line_length_pclk = 0xff,
  .frame_length_lines = 0xff,
};

static struct msm_sensor_exp_gain_info_t exp_gain_info = {
  .coarse_int_time_addr = 0xff,
  .global_gain_addr = 0xff,
  .vert_offset = 8,
};

static sensor_aec_data_t aec_info = {
  .max_gain = 3.984375,
  .max_linecount = 1477,
};

static sensor_lens_info_t default_lens_info = {
  .focal_length = 3.16,
  .pix_size = 1.12,
  .f_number = 2.4,
  .total_f_dist = 1.2,
  .hor_view_angle = 56.0,
  .ver_view_angle = 42.0,
};

#ifndef VFE_40
static struct csi_lane_params_t csi_lane_params = {
  .csi_lane_assign = 0x0, // 4320,
  .csi_lane_mask = 0x3,
  .csi_if = 1,
  .csid_core = {0},
  .csi_phy_sel = 0,
};
#else
#error "NDM - we don't support this hardware"
static struct csi_lane_params_t csi_lane_params = {
  .csi_lane_assign = 0x4320,
  .csi_lane_mask = 0x3,
  .csi_if = 1,
  .csid_core = {0},
  .csi_phy_sel = 0,
};
#endif

static struct msm_camera_i2c_reg_array init_reg_array0[] = {
        {0xfe, 0x00, 0x00},
};

static struct msm_camera_i2c_reg_array init_reg_array1[] = {

//Actual_window_size=1280*720
//MCLK=24MHz,MIPI_clock=456Mhz,row_time=43.28us
//pixel_line=2467,frame_line=768
//////////////////////////////////////////
/////////////////////// SYS //////////////
//////////////////////////////////////////
  {0xf7, 0x01, 0x00},
  {0xf8, 0x12, 0x00}, // was 0x12 or 24*19
  {0xf9, 0x0a, 0x00},
  {0xfc, 0x0e, 0x00},
  {0xfd, 0x00, 0x00},
/////////////////////////////////
/////////   Analog   ////////////
/////////////////////////////////
  {0xfe, 0x00, 0x00},
  {0x03, 0x02, 0x00}, // Exposure[12:8]
  {0x04, 0xb5, 0x00}, // Exposure[7:0]
  {0x05, 0x02, 0x00},
  {0x06, 0x48, 0x00},
  {0x07, 0x00, 0x00}, //vb
  {0x08, 0x10, 0x00},
  {0x09, 0x00, 0x00}, //row start
  {0x0a, 0x20, 0x00}, //
  {0x0b, 0x00, 0x00}, //col start
  {0x0c, 0x20, 0x00},
  {0x0d, 0x02, 0x00}, //height  736
  {0x0e, 0xe0, 0x00},
  {0x0f, 0x05, 0x00}, //width   1296
  {0x10, 0x10, 0x00},
  {0x17, 0xd5, 0x00}, //cfa
  {0x18, 0x02, 0x00},
  {0x19, 0x2b, 0x00},
  {0x1b, 0xf1, 0x00},
  {0x1c, 0x2c, 0x00},
  {0x1e, 0x70, 0x00},
  {0x1f, 0xa0, 0x00}, //gamma
  {0x20, 0xca, 0x00}, //gamma offset
  {0x25, 0xc1, 0x00}, //[7:6]txl_s_mode
  {0x26, 0x0e, 0x00},
  {0x27, 0x20, 0x00}, //[5:4]txl_drv_mode
  {0x29, 0x40, 0x00}, //EQP
  {0x2b, 0x88, 0x00}, //init_rampb_mode
  {0x2f, 0xf4, 0x00},
  {0x32, 0x1a, 0x00},
  {0x37, 0xff, 0x00},
  {0x38, 0x03, 0x00}, //head_acc_en off
  {0xcd, 0xa8, 0x00},
  {0xce, 0xcd, 0x00},
  {0xd1, 0x9d, 0x00}, //vref
  {0xd3, 0x33, 0x00},
  {0xd8, 0xa0, 0x00},
  {0xd9, 0xfa, 0x00},
  {0xda, 0xc3, 0x00}, //[7:6] ctdsun
  {0xe1, 0x24, 0x00}, //[3:0]post tx width x4
  {0xe3, 0x01, 0x00},
  {0xe4, 0xf8, 0x00},
  {0xe6, 0x20, 0x00}, //ramps_offset
  {0xe7, 0xca, 0x00}, //[7:6]wen_U
  {0xe8, 0x00, 0x00},
  {0xe9, 0x00, 0x00},
  {0xea, 0x00, 0x00},
  {0xeb, 0x00, 0x00},
/////////////////////////////////
////////// ISP //////////////////
/////////////////////////////////
  {0xfe, 0x00, 0x00},
  {0x80, 0x50, 0x00},
  {0x88, 0x03, 0x00},
  {0x89, 0x03, 0x00},
  {0x8a, 0xb3, 0x00},
  {0x8d, 0x03, 0x00},
/////////////////////////////////
////////// crop /////////////////
/////////////////////////////////
  {0xfe, 0x00, 0x00},
  {0x90, 0x01, 0x00}, 
  {0x91, 0x00, 0x00},
  {0x92, 0x02, 0x00},
  {0x93, 0x00, 0x00},
  {0x94, 0x01, 0x00},
  {0x95, 0x02, 0x00},
  {0x96, 0xd0, 0x00},   //720
  {0x97, 0x05, 0x00},
  {0x98, 0x00, 0x00},   //1280
/////////////////////////////////
//////////   BLK      ///////////
/////////////////////////////////
  {0xfe, 0x00, 0x00},
  {0x40, 0x22, 0x00},
  {0x4e, 0x3c, 0x00},
  {0x4f, 0x00, 0x00},
  {0x60, 0x00, 0x00},
  {0x61, 0x80, 0x00},
/////////////////////////////////
/////// gain ////////////////////
/////////////////////////////////
  {0xfe, 0x00, 0x00}, 
  {0xb0, 0x48, 0x00}, //global gain      
  {0xb1, 0x01, 0x00},
  {0xb2, 0x00, 0x00},
  {0xb3, 0x80, 0x00}, // AWB_R_gain
  {0xb4, 0x40, 0x00}, // AWB_G_gain
  {0xb5, 0x80, 0x00}, // AWB_B_gain
  {0xb6, 0x00, 0x00},
/////////////////////////////////
/////// dd //////////////////////
/////////////////////////////////
  {0xfe, 0x02, 0x00},
  {0x80, 0x08, 0x00},
  {0x83, 0x60, 0x00},
  {0x84, 0x80, 0x00},
  {0x85, 0x60, 0x00},
  {0x86, 0x30, 0x00},
  {0x87, 0xa0, 0x00},
  {0x88, 0x80, 0x00},
  {0x89, 0x40, 0x00},
/////////////////////////////////
/////// ASDE ////////////////////
/////////////////////////////////
  {0xfe, 0x02, 0x00},
  {0xa0, 0x48, 0x00},
  {0xa3, 0x45, 0x00},
/////////////////////////////////
////////// dark sun /////////////
/////////////////////////////////
  {0xfe, 0x00, 0x00}, 
  {0x68, 0xc7, 0x00}, //87
  {0x6c, 0x83, 0x00},
  {0x6e, 0xc4, 0x00},
/////////////////////////////////
///////// WB offset /////////////
/////////////////////////////////
  {0xfe, 0x00, 0x00}, 
  {0x3c, 0x00, 0x00},
  {0x3d, 0x00, 0x00},
  {0x3f, 0x00, 0x00},
/////////////////////////////////
/////// mipi ////////////////////
/////////////////////////////////
  {0xfe, 0x03, 0x00},
  {0x01, 0x03, 0x00},
  {0x02, 0x33, 0x00},
  {0x03, 0x93, 0x00},
  {0x04, 0x04, 0x00},
  {0x05, 0x00, 0x00},
  {0x06, 0x80, 0x00},
  {0x10, 0x80, 0x00},
  {0x11, 0x2b, 0x00},
  {0x12, 0x40, 0x00},
  {0x13, 0x06, 0x00},
  {0x15, 0x02, 0x00},
  {0x21, 0x10, 0x00},
  {0x22, 0x03, 0x00},
  {0x23, 0x30, 0x00},
  {0x24, 0x02, 0x00},
  {0x25, 0x15, 0x00},
  {0x26, 0x05, 0x00},
  {0x29, 0x03, 0x00},
  {0x2a, 0x0a, 0x00},
  {0x2b, 0x05, 0x00},
  {0xfe, 0x00, 0x00},

};

static struct msm_camera_i2c_reg_setting init_reg_setting[] = {
  {
    .reg_setting = init_reg_array0,
    .size = ARRAY_SIZE(init_reg_array0),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 50,
  },
  {
    .reg_setting = init_reg_array1,
    .size = ARRAY_SIZE(init_reg_array1),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
  },
};

static struct sensor_lib_reg_settings_array init_settings_array = {
  .reg_settings = init_reg_setting,
  .size = 2,
};

static struct msm_camera_i2c_reg_array start_reg_array[] = {
	{0xfe, 0x03, 0x00},
	{0x10, 0x90, 0x00},
	{0xfe, 0x00, 0x00},
};

static  struct msm_camera_i2c_reg_setting start_settings = {
  .reg_setting = start_reg_array,
  .size = ARRAY_SIZE(start_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_i2c_reg_array stop_reg_array[] = {
        {0xfe, 0x03, 0x00},
        {0x10, 0x80, 0x00},
        {0xfe, 0x00, 0x00},
};

static struct msm_camera_i2c_reg_setting stop_settings = {
  .reg_setting = stop_reg_array,
  .size = ARRAY_SIZE(stop_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_i2c_reg_array groupon_reg_array[] = {
        {0xfe, 0x00, 0x00},
};

static struct msm_camera_i2c_reg_setting groupon_settings = {
  .reg_setting = groupon_reg_array,
  .size = ARRAY_SIZE(groupon_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_i2c_reg_array groupoff_reg_array[] = {
        {0xfe, 0x00, 0x00},
};

static struct msm_camera_i2c_reg_setting groupoff_settings = {
  .reg_setting = groupoff_reg_array,
  .size = ARRAY_SIZE(groupoff_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_csid_vc_cfg ov8856_f8v05a_cid_cfg[] = {
  {0, CSI_RAW10, CSI_DECODE_10BIT},
  {1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params ov8856_f8v05a_csi_params = {
  .csid_params = {
    .lane_cnt = 1, // 2,
    .lut_params = {
      .num_cid = ARRAY_SIZE(ov8856_f8v05a_cid_cfg),
      .vc_cfg = {
         &ov8856_f8v05a_cid_cfg[0],
         &ov8856_f8v05a_cid_cfg[1],
      },
    },
  },
  .csiphy_params = {
    .lane_cnt = 1, // NDM 2,
    .settle_cnt = 0x7, // NDM - why not? 1b,
  },
};

static struct sensor_pix_fmt_info_t ov8856_f8v05a_pix_fmt0_fourcc[] = {
  { V4L2_PIX_FMT_SBGGR10 },
};

static struct sensor_pix_fmt_info_t ov8856_f8v05a_pix_fmt1_fourcc[] = {
  { MSM_V4L2_PIX_FMT_META },
};

static sensor_stream_info_t ov8856_f8v05a_stream_info[] = {
  {1, &ov8856_f8v05a_cid_cfg[0], ov8856_f8v05a_pix_fmt0_fourcc},
  {1, &ov8856_f8v05a_cid_cfg[1], ov8856_f8v05a_pix_fmt1_fourcc},
};

static sensor_stream_info_array_t ov8856_f8v05a_stream_info_array = {
  .sensor_stream_info = ov8856_f8v05a_stream_info,
  .size = ARRAY_SIZE(ov8856_f8v05a_stream_info),
};

static struct msm_camera_i2c_reg_array res0_reg_array[] = {
        {0xfe, 0x00, 0x00},  // Set page to 0 (no-op)
};
static struct msm_camera_i2c_reg_setting res_settings[] = {
  {
    .reg_setting = res0_reg_array,
    .size = ARRAY_SIZE(res0_reg_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
  },
};

static struct sensor_lib_reg_settings_array res_settings_array = {
  .reg_settings = res_settings,
  .size = ARRAY_SIZE(res_settings),
};

static struct msm_camera_csi2_params *csi_params[] = {
  &ov8856_f8v05a_csi_params, /* RES 0*/
};

static struct sensor_lib_csi_params_array csi_params_array = {
  .csi2_params = &csi_params[0],
  .size = ARRAY_SIZE(csi_params),
};

static struct sensor_crop_parms_t crop_params[] = {
  {0, 0, 0, 0}, /* RES 0 */
};
static struct sensor_lib_crop_params_array crop_params_array = {
  .crop_params = crop_params,
  .size = ARRAY_SIZE(crop_params),
};

static struct sensor_lib_out_info_t sensor_out_info[] = {
  {
    .x_output = 1280, // 3264,
    .y_output = 720, // 2448,
    .line_length_pclk = 1296, // 880, // 1932,
    .frame_length_lines = 768, // 2482,
    .vt_pixel_clk = 45600000,  // Fake pseudo-clock
    .op_pixel_clk = 45600000,
    .binning_factor = 1,
    .max_fps = 30.0,
    .min_fps = 7.5, 
    .mode = SENSOR_DEFAULT_MODE,
  },
};

static struct sensor_lib_out_info_array out_info_array = {
  .out_info = sensor_out_info,
  .size = ARRAY_SIZE(sensor_out_info),
};

static sensor_res_cfg_type_t ov8856_f8v05a_res_cfg[] = {
  SENSOR_SET_STOP_STREAM,
  SENSOR_SET_NEW_RESOLUTION, /* set stream config */
  SENSOR_SET_CSIPHY_CFG,
  SENSOR_SET_CSID_CFG,
  SENSOR_LOAD_CHROMATIX, /* set chromatix prt */
  SENSOR_SEND_EVENT, /* send event */
  SENSOR_SET_START_STREAM,
};

static struct sensor_res_cfg_table_t ov8856_f8v05a_res_table = {
  .res_cfg_type = ov8856_f8v05a_res_cfg,
  .size = ARRAY_SIZE(ov8856_f8v05a_res_cfg),
};

static struct sensor_lib_chromatix_t ov8856_f8v05a_chromatix[] = {
  {
    .common_chromatix = OV8856_F8V05A_LOAD_CHROMATIX(common),
    .camera_preview_chromatix = OV8856_F8V05A_LOAD_CHROMATIX(zsl), /* RES0 */
    .camera_snapshot_chromatix = OV8856_F8V05A_LOAD_CHROMATIX(snapshot), /* RES0 */
    .camcorder_chromatix = OV8856_F8V05A_LOAD_CHROMATIX(default_video), /* RES0 */
    .liveshot_chromatix =  OV8856_F8V05A_LOAD_CHROMATIX(liveshot), /* RES0 */
  },
};

static struct sensor_lib_chromatix_array ov8856_f8v05a_lib_chromatix_array = {
  .sensor_lib_chromatix = ov8856_f8v05a_chromatix,
  .size = ARRAY_SIZE(ov8856_f8v05a_chromatix),
};

/*===========================================================================
 * FUNCTION    - ov8856_f8v05a_real_to_register_gain -
 *
 * DESCRIPTION:
 *==========================================================================*/
// Convert to fixed point u8 with 6 places after the decimal (2^6 = 64)
// as defined by the camera specs for the format of gain
#define GAIN_TO_FIXED_POINT(gain) ((uint8_t)((gain) * (64)))
// 1/64 = 0.015625
#define FIXED_POINT_TO_GAIN(fp) ((float)((fp) * (0.015625)))
static uint8_t real_to_register_gain(float gain)
{
  const float max_gain = FIXED_POINT_TO_GAIN(0xFF);
  const float min_gain = FIXED_POINT_TO_GAIN(0x01);
  if(gain > max_gain)
  {
    gain = max_gain;
  }
  else if(gain < min_gain)
  {
    gain = min_gain;
  }

  return GAIN_TO_FIXED_POINT(gain);
}

/*===========================================================================
 * FUNCTION    - ov8856_f8v05a_register_to_real_gain -
 *
 * DESCRIPTION:
 *==========================================================================*/
static float register_to_real_gain(uint8_t reg_gain)
{
  const uint8_t min_gain = 0x01;
  if(reg_gain < min_gain)
  {
    reg_gain = min_gain;
  }

  return FIXED_POINT_TO_GAIN(reg_gain);
}

/*===========================================================================
 * FUNCTION    - ov8856_f8v05a_calculate_exposure -
 *
 * DESCRIPTION:
 *==========================================================================*/
static int32_t ov8856_f8v05a_calculate_exposure(float real_gain,
  uint16_t line_count, sensor_exposure_info_t *exp_info)
{
  if (!exp_info) {
    return -1;
  }

  exp_info->reg_gain = real_to_register_gain(real_gain);
  exp_info->sensor_real_gain = register_to_real_gain(exp_info->reg_gain);
  exp_info->digital_gain = real_gain / exp_info->sensor_real_gain;

  if(!isfinite(exp_info->digital_gain))
  {
    exp_info->digital_gain = 1;
  }

  if(line_count > 1477)
  {
    line_count = 1477;
  }

  exp_info->line_count = line_count;
  exp_info->sensor_digital_gain = 0x1;
  return 0;
}

/*===========================================================================
 * FUNCTION    - ov8856_f8v05a_fill_exposure_array -
 *
 * DESCRIPTION:
 *==========================================================================*/
static int32_t ov8856_f8v05a_fill_exposure_array(uint16_t gain, 
                                                 uint32_t line,
                                                 uint32_t fl_lines, 
                                                 int32_t luma_avg, 
                                                 uint32_t fgain,
                                                 struct msm_camera_i2c_reg_setting* reg_setting)
{
  uint16_t reg_count = 0;

  if (!reg_setting) {
    return -1;
  }

  if(gain > 0xFF)
  {
    gain = 0xFF;
  }
  else if(gain < 0x01)
  {
    gain = 0x01;
  }

  if(line > 1477)
  {
    line = 1477;
  }

  reg_setting->reg_setting[reg_count].reg_addr = 0xfe;
  reg_setting->reg_setting[reg_count].reg_data = 0x00;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0x03;
  reg_setting->reg_setting[reg_count].reg_data = (line & 0xFF00) >> 8;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0x04;
  reg_setting->reg_setting[reg_count].reg_data = line & 0xFF;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0xb0;
  reg_setting->reg_setting[reg_count].reg_data = gain & 0xFF;
  reg_count++;

  reg_setting->size = reg_count;
  reg_setting->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
  reg_setting->data_type = MSM_CAMERA_I2C_BYTE_DATA;
  reg_setting->delay = 0;

  return 0;
}

static int sensor_fill_awb_array(unsigned short awb_gain_r, 
                                 unsigned short awb_gain_b,
                                 struct msm_camera_i2c_seq_reg_setting* reg_setting)
{
  uint16_t reg_count = 0;

  if (!reg_setting) {
    return -1;
  }

  const uint16_t max_gain = 0xFF;
  const uint16_t min_gain = 0x01;
  if(awb_gain_r > max_gain)
  {
    awb_gain_r = max_gain;
  }
  else if(awb_gain_r < min_gain)
  {
    awb_gain_r = min_gain;
  }

  if(awb_gain_b > max_gain)
  {
    awb_gain_b = max_gain;
  }
  else if(awb_gain_b < min_gain)
  {
    awb_gain_b = min_gain;
  }

  reg_setting->reg_setting[reg_count].reg_addr = 0xfe;
  reg_setting->reg_setting[reg_count].reg_data[0] = 0x00;
  reg_setting->reg_setting[reg_count].reg_data_size = 1;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0xb3;
  reg_setting->reg_setting[reg_count].reg_data[0] = awb_gain_r & 0xFF;
  reg_setting->reg_setting[reg_count].reg_data_size = 1;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0xb5;
  reg_setting->reg_setting[reg_count].reg_data[0] = awb_gain_b & 0xFF;
  reg_setting->reg_setting[reg_count].reg_data_size = 1;
  reg_count++;

  reg_setting->size = reg_count;
  reg_setting->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
  reg_setting->delay = 0;

  return 0;
}

static sensor_exposure_table_t ov8856_f8v05a_expsoure_tbl = {
  .sensor_calculate_exposure = ov8856_f8v05a_calculate_exposure,
  .sensor_fill_exposure_array = ov8856_f8v05a_fill_exposure_array,
};

static sensor_video_hdr_table_t video_hdr_tbl = {
  .sensor_fill_awb_array = sensor_fill_awb_array,
  .awb_table_size = 2, // Maybe should be 1?
  .video_hdr_capability = 0, // (1<<8) | (1<<20),  maybe 0x100?
};

static sensor_lib_t sensor_lib_ptr = {
  /* sensor actuator name */
//  .actuator_name = "dw9714_f8v05a",
  /* sensor slave info */
  .sensor_slave_info = &sensor_slave_info,
  /* sensor init params */
  .sensor_init_params = &sensor_init_params,
  /* sensor output settings */
  .sensor_output = &sensor_output,
  /* sensor eeprom name */
//  .eeprom_name = "truly_ov8856",
  /* sensor output register address */
  .output_reg_addr = &output_reg_addr,
  /* sensor exposure gain register address */
  .exp_gain_info = &exp_gain_info,
  /* sensor aec info */
  .aec_info = &aec_info,
  /* sensor snapshot exposure wait frames info */
  .snapshot_exp_wait_frames = 1,
  /* number of frames to skip after start stream */
  .sensor_num_frame_skip = 1,
  /* number of frames to skip after start HDR stream */
  .sensor_num_HDR_frame_skip = 2,
  /* sensor pipeline immediate delay */
  .sensor_max_pipeline_frame_delay = 2,
  /* sensor exposure table size */
  .exposure_table_size = 10,
  /* sensor lens info */
  .default_lens_info = &default_lens_info,
  /* csi lane params */
  .csi_lane_params = &csi_lane_params,
  /* csi cid params */
  .csi_cid_params = ov8856_f8v05a_cid_cfg,
  /* csi csid params array size */
  .csi_cid_params_size = ARRAY_SIZE(ov8856_f8v05a_cid_cfg),
  /* init settings */
  .init_settings_array = &init_settings_array,
  /* start settings */
  .start_settings = &start_settings,
  /* stop settings */
  .stop_settings = &stop_settings,
  /* group on settings */
  .groupon_settings = &groupon_settings,
  /* group off settings */
  .groupoff_settings = &groupoff_settings,
  /* resolution cfg table */
  .sensor_res_cfg_table = &ov8856_f8v05a_res_table,
  /* res settings */
  .res_settings_array = &res_settings_array,
  /* out info array */
  .out_info_array = &out_info_array,
  /* crop params array */
  .crop_params_array = &crop_params_array,
  /* csi params array */
  .csi_params_array = &csi_params_array,
  /* sensor port info array */
  .sensor_stream_info_array = &ov8856_f8v05a_stream_info_array,
  /* exposure funtion table */
  .exposure_func_table = &ov8856_f8v05a_expsoure_tbl,
  /* chromatix array */
  .chromatix_array = &ov8856_f8v05a_lib_chromatix_array,
  /* sensor pipeline immediate delay */
  .sensor_max_immediate_frame_delay = 2,
  .sync_exp_gain = 1,

  /* video hdr func table */
  .video_hdr_awb_lsc_func_table = &video_hdr_tbl,
};

/*===========================================================================
 * FUNCTION    - ov8856_f8v05a_open_lib -
 *
 * DESCRIPTION:
 *==========================================================================*/
void *ov8856_f8v05a_open_lib(void)
{
  return &sensor_lib_ptr;
}
