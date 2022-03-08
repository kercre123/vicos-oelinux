/*============================================================================

  Copyrigh(c) 2014 Qualcomm Technologies,Inc. All Rights Reserved.
  Qualcomm Technologies Proprietary and Confidential.

============================================================================*/
#include <stdio.h>
#include "sensor_lib.h"
//#include "camera_dbg.h"
//#include <utils/Log.h>

#define SENSOR_MODEL_NO_BF2253L "bf2253L"
#define BF2253L_LOAD_CHROMATIX(n) \
  "libchromatix_"SENSOR_MODEL_NO_BF2253L"_"#n".so"

//#define BF2253LMIPI_SUB
#define MaxGainIndex (65)

static sensor_lib_t sensor_lib_ptr;
static struct msm_sensor_power_setting power_setting[] = {
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_STANDBY,
    .config_val = GPIO_OUT_HIGH,
    .delay = 10,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_RESET,
    .config_val = GPIO_OUT_LOW,
    .delay = 10,
  },
  {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VIO,
    .config_val = 0,
    .delay = 10,
  },
  {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VDIG,
    .config_val = 0,
    .delay = 10,
  }, 
  {
    .seq_type = SENSOR_CLK,
    .seq_val = SENSOR_CAM_MCLK,
    .config_val = 24000000,
    .delay = 5,
  },   
  {
    .seq_type = SENSOR_VREG,
    .seq_val = CAM_VANA,
    .config_val = 0,
    .delay = 10,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_STANDBY,
    .config_val = GPIO_OUT_LOW, 
    .delay = 10,
  },
  {
    .seq_type = SENSOR_GPIO,
    .seq_val = SENSOR_GPIO_RESET,
    .config_val = GPIO_OUT_HIGH,
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
  #ifdef BF2253LMIPI_SUB
  .camera_id = CAMERA_1,
  #else  
  .camera_id = CAMERA_0,
  #endif
  /* sensor slave address */
  .slave_addr = 0xdc,
  /* sensor i2c frequency*/
  .i2c_freq_mode = I2C_FAST_MODE,
  /* sensor address type */
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  /* sensor id info*/
  .sensor_id_info = {
    /* sensor id register address */
    .sensor_id_reg_addr = 0xfc,
    /* sensor id */
    .sensor_id = 0x2253,
  },
  /* power up / down setting */
  .power_setting_array = {
    .power_setting = power_setting,
    .size = ARRAY_SIZE(power_setting),
  },
};

static struct msm_sensor_init_params sensor_init_params = {
  #ifdef BF2253LMIPI_SUB
  .modes_supported = 0,
  .position = 1,
  .sensor_mount_angle = 270,
  #else  
  .modes_supported = 0,
  .position = 0,
  .sensor_mount_angle = 90,
  #endif  
};

static sensor_output_t sensor_output = {
  .output_format = SENSOR_BAYER,
  .connection_mode = SENSOR_MIPI_CSI,
  .raw_output = SENSOR_10_BIT_DIRECT,
};

static struct msm_sensor_output_reg_addr_t output_reg_addr = {
  .x_output = 0xff,
  .y_output = 0xff,
  .line_length_pclk = 0xff,
  .frame_length_lines = 0xff,
};

static struct msm_sensor_exp_gain_info_t exp_gain_info = {
  .coarse_int_time_addr = 0x6b,
  .global_gain_addr = 0x6a,
  .vert_offset = 1,
};

static sensor_aec_data_t aec_info = {
  .max_gain = 6.0,
  .max_linecount = 65534,
};

static sensor_lens_info_t default_lens_info = {
  .focal_length = 2.93,
  .pix_size = 1.75,
  .f_number = 2.8,
  .total_f_dist = 1.2,
  .hor_view_angle = 54.8,
  .ver_view_angle = 42.5,
};

#ifdef BF2253LMIPI_SUB
static struct csi_lane_params_t csi_lane_params = {
  .csi_lane_assign = 0x0004,
  .csi_lane_mask = 0x18,
  .csi_if = 1,
  .csid_core = {0},
  .csi_phy_sel = 0,
};
#else
static struct csi_lane_params_t csi_lane_params = {
  .csi_lane_assign = 0x4320,
  .csi_lane_mask = 0x3,
  .csi_if = 1,
  .csid_core = {0},
  .csi_phy_sel = 0,
};
#endif

static struct msm_camera_i2c_reg_array init_reg_array0[] = {
//;==================INI==================
//Product Ver:VAI01
//;;Output Detail:
//;;XCLK:24 MHz
//;;PCLK:66 MHz
//;;MipiCLK:660 MHz
//;;FrameW:1780
//;;FrameH:1236

	//{0xf2, 0x01},/0xf2[0]:1,reset
	{0x00, 0x22},// Mirror and invert pixels
	{0xe1, 0x06},
	{0xe2, 0x06},
	{0xe3, 0x0e},
	{0xe4, 0x60},
	{0xe5, 0x67},
	{0xe6, 0x02},
	{0xe8, 0x94},//0x84  //2020.11.12 //ryx 
	{0x01, 0x14},
	{0x03, 0x98},
	{0x27, 0x21},
	{0x29, 0x20},
	{0x59, 0x10},
	{0x5a, 0x10},
	{0x5c, 0x11},
	{0x5d, 0x73},
	{0x6a, 0x2f},
	{0x6b, 0x0e},
	{0x6c, 0x7e},
	{0x6f, 0x10},
	{0x70, 0x08},
	{0x71, 0x05},
	{0x72, 0x10},
	{0x73, 0x08},
	{0x74, 0x05},
	{0x75, 0x06},
	{0x76, 0x20},
	{0x77, 0x03},
	{0x78, 0x0e},
	{0x79, 0x08},
	{0x00, 0x22},// Mirror and invert pixels		
};

static struct msm_camera_i2c_reg_setting init_reg_setting[] = {
  {
    .reg_setting = init_reg_array0,
    .size = ARRAY_SIZE(init_reg_array0),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 1,//50,
  },

};

static struct sensor_lib_reg_settings_array init_settings_array = {
  .reg_settings = init_reg_setting,
  .size = 1,
};

static struct msm_camera_i2c_reg_array start_reg_array[] = {
	{0xe0, 0x00},
	{0x00, 0x22}, // Mirror and invert pixels
	{0x01, 0x14},

};

static struct msm_camera_i2c_reg_setting start_settings = {
  .reg_setting = start_reg_array,
  .size = ARRAY_SIZE(start_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 10,
};

static struct msm_camera_i2c_reg_array stop_reg_array[] = {
	{0xe0, 0x01},
	{0x01, 0x1c},
	
};

static struct msm_camera_i2c_reg_setting stop_settings = {
  .reg_setting = stop_reg_array,
  .size = ARRAY_SIZE(stop_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 10,
};

static struct    msm_camera_i2c_reg_array groupon_reg_array[] = {
    {0xff, 0xff},
};

static struct msm_camera_i2c_reg_setting groupon_settings = {
  .reg_setting = groupon_reg_array,
  .size = ARRAY_SIZE(groupon_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_i2c_reg_array groupoff_reg_array[] = {
    {0xff, 0xff},
};

static struct msm_camera_i2c_reg_setting groupoff_settings = {
  .reg_setting = groupoff_reg_array,
  .size = ARRAY_SIZE(groupoff_reg_array),
  .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
  .data_type = MSM_CAMERA_I2C_BYTE_DATA,
  .delay = 0,
};

static struct msm_camera_csid_vc_cfg bf2253L_cid_cfg[] = {
  {0, CSI_RAW10, CSI_DECODE_10BIT},
  {1, CSI_EMBED_DATA, CSI_DECODE_10BIT},
};

static struct msm_camera_csi2_params bf2253L_csi_params = {
  .csid_params = {
    .lane_cnt = 1,
    .lut_params = {
      .num_cid = ARRAY_SIZE(bf2253L_cid_cfg),
      .vc_cfg = {
         &bf2253L_cid_cfg[0],
         &bf2253L_cid_cfg[1],
      },
    },
  },
  .csiphy_params = {
    .lane_cnt = 1,
    .settle_cnt = 0x14,//120ns
#ifndef VFE_40
    .combo_mode = 1,
#endif
  },
};

struct sensor_pix_fmt_info_t rgb10[] =
{  //only a simbol rgb10
    {V4L2_PIX_FMT_SBGGR10},
};

struct sensor_pix_fmt_info_t meta[] =
{//only a simbol meta
    {MSM_V4L2_PIX_FMT_META},
};

static sensor_stream_info_t bf2253L_stream_info[] = {
  {1, &bf2253L_cid_cfg[0], rgb10},
  {1, &bf2253L_cid_cfg[1], meta},
};

static sensor_stream_info_array_t bf2253L_stream_info_array = {
  .sensor_stream_info = bf2253L_stream_info,
  .size = ARRAY_SIZE(bf2253L_stream_info),
};

static struct msm_camera_i2c_reg_array res0_reg_array[] = {
/* lane snap */
  {0xff,0xff},
};

static struct msm_camera_i2c_reg_array res1_reg_array[] = {
/*  preveiw */
  {0xff,0xff},
};

static struct msm_camera_i2c_reg_setting res_settings[] = {
  {//capture
    .reg_setting = res0_reg_array,
    .size = ARRAY_SIZE(res0_reg_array),
    .addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
    .data_type = MSM_CAMERA_I2C_BYTE_DATA,
    .delay = 0,
  },
  {//preview
    .reg_setting = res1_reg_array,
    .size = ARRAY_SIZE(res1_reg_array),
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
  &bf2253L_csi_params, /* RES 0*/
  &bf2253L_csi_params, /* RES 1*/
};

static struct sensor_lib_csi_params_array csi_params_array = {
  .csi2_params = &csi_params[0],
  .size = ARRAY_SIZE(csi_params),
};

static struct sensor_crop_parms_t crop_params[] = {
  {0, 0, 0, 0}, /* RES 0 */
  {0, 0, 0, 0}, /* RES 1 */
};

static struct sensor_lib_crop_params_array crop_params_array = {
  .crop_params = crop_params,
  .size = ARRAY_SIZE(crop_params),
};

static struct sensor_lib_out_info_t sensor_out_info[] = {
  {
/* For SNAPSHOT */
    .x_output = 1600,
    .y_output = 1200,
    .line_length_pclk = 1780,   
    .frame_length_lines = 1236, 
    .vt_pixel_clk = 66000000,     
    .op_pixel_clk = 66000000,    
    .binning_factor = 1,
    .max_fps = 30.0,
    .min_fps = 8,
    .mode = SENSOR_DEFAULT_MODE,
  },
/* For PREVIEW */
  {
    .x_output = 1600,
    .y_output = 1200,
    .line_length_pclk = 1780,   
    .frame_length_lines = 1236, 
    .vt_pixel_clk = 66000000,     
    .op_pixel_clk = 66000000,    
    .binning_factor = 1,
    .max_fps = 30.0,
    .min_fps = 8,
    .mode = SENSOR_DEFAULT_MODE,
  },
};

static struct sensor_lib_out_info_array out_info_array = {
  .out_info = sensor_out_info,
  .size = ARRAY_SIZE(sensor_out_info),
};

static sensor_res_cfg_type_t bf2253L_res_cfg[] = {
  SENSOR_SET_STOP_STREAM,
  SENSOR_SET_NEW_RESOLUTION, /* setstream config */
  SENSOR_SET_CSIPHY_CFG,
  SENSOR_SET_CSID_CFG,
  SENSOR_LOAD_CHROMATIX, /* setchromatix prt */
  SENSOR_SEND_EVENT, /* send event */
  SENSOR_SET_START_STREAM,
};

static struct sensor_res_cfg_table_t bf2253L_res_table = {
  .res_cfg_type = bf2253L_res_cfg,
  .size = ARRAY_SIZE(bf2253L_res_cfg),
};

static struct sensor_lib_chromatix_t bf2253L_chromatix[] = {
  {
    .common_chromatix = BF2253L_LOAD_CHROMATIX(common),
    .camera_preview_chromatix = BF2253L_LOAD_CHROMATIX(preview), /* RES0 */
    .camera_snapshot_chromatix = BF2253L_LOAD_CHROMATIX(preview), /* RES0 */
    .camcorder_chromatix = BF2253L_LOAD_CHROMATIX(preview), /* RES0 */
  },
  {
    .common_chromatix = BF2253L_LOAD_CHROMATIX(common),
    .camera_preview_chromatix = BF2253L_LOAD_CHROMATIX(preview), /* RES0 */
    .camera_snapshot_chromatix = BF2253L_LOAD_CHROMATIX(preview), /* RES0 */
    .camcorder_chromatix = BF2253L_LOAD_CHROMATIX(preview), /* RES0 */
  },
};

static struct sensor_lib_chromatix_array bf2253L_lib_chromatix_array = {
  .sensor_lib_chromatix = bf2253L_chromatix,
  .size = ARRAY_SIZE(bf2253L_chromatix),
};
// Gain Index
static float sensorGainMapping[MaxGainIndex][2] ={
    {1.000000  ,15},
    {1.062099  ,16},
    {1.122899  ,17},
    {1.185477  ,18},
    {1.246952  ,19},
    {1.310529  ,20},
    {1.370253  ,21},
    {1.433147  ,22},
    {1.491239  ,23},
    {1.556268  ,24},
    {1.613361  ,25},
    {1.675742  ,26},
    {1.733732  ,27},
    {1.795754  ,28},
    {1.852078  ,29},
    {1.913912  ,30},
    {1.975430  ,31},
    {2.095126  ,32},
    {2.219256  ,33},
    {2.338661  ,34},
    {2.460108  ,35},
    {2.579394  ,36},
    {2.699902  ,37},
    {2.817171  ,38},
    {2.935996  ,39},
    {3.057435  ,40},
    {3.172791  ,41},
    {3.293828  ,42},
    {3.411311  ,43},
    {3.525240  ,44},
    {3.642997  ,45},
    {3.755935  ,46},
    {3.869326  ,47},
    {4.107573  ,48},
    {4.337166  ,49},
    {4.568417  ,50},
    {4.790962  ,51},
    {5.025270  ,52},
    {5.245363  ,53},
    {5.469403  ,54},
    {5.694263  ,55},
    {5.903917  ,56},
    {6.112750  ,57},
    {6.341122  ,58},
    {6.555995  ,59},
    {6.765162  ,60},
    {6.981402  ,61},
    {7.186818  ,62},
    {7.386690  ,63},
    {7.606775  ,64},
    {7.812362  ,65},
    {8.023775  ,66},
    {8.241425  ,67},
    {8.445218  ,68},
    {8.651044  ,69},
    {8.852625  ,70},
    {9.062723  ,71},
    {9.260074  ,72},
    {9.438042  ,73},
    {9.659058  ,74},
    {9.836786  ,75},
    {10.032062 ,76},
    {10.228218 ,77},
    {10.413378 ,78},
    {10.601478 ,79},
};
static uint16_t gain2reg(float gain)
{
    uint16_t iI;
    uint16_t ret;

    for (iI = 0; iI < (MaxGainIndex-1); iI++) {
    	if((gain >= sensorGainMapping[iI][0])&&(gain < sensorGainMapping[iI+1][0]))
    	{
	    	if((gain-sensorGainMapping[iI][0]) <= (sensorGainMapping[iI+1][0]-gain))
	    		ret = (int16_t)sensorGainMapping[iI][1];
	    	else
	    		ret = (int16_t)sensorGainMapping[iI+1][1];
       break;
    	}
    }
    return ret;
}

/*===========================================================================
 * FUNCTION - bf2253L_rio6_real_to_register_gain -
 *
 * DESCRIPTION:
 *==========================================================================*/
static uint16_t bf2253L_real_to_register_gain(float gain) 
{
  uint16_t reg_gain = 0;
//  ALOGE("111#######float_gain@@ = %f", gain);
  if (gain < 1.0)
    gain = 1.0;
  if (gain > 6)
    gain = 6;

  reg_gain = gain2reg(gain);

  return reg_gain;
}

/*===========================================================================
 * FUNCTION - bf2253L_rio6_register_to_real_gain -
 *
 * DESCRIPTION:
 *==========================================================================*/
static float bf2253L_register_to_real_gain(uint16_t reg_gain)
{
    uint16_t iI;
    for (iI = 0; iI < MaxGainIndex; iI++) {
        if(reg_gain <= (int16_t)sensorGainMapping[iI][1]){
            break;
        }
    }
    return sensorGainMapping[iI][0];
}

/*===========================================================================
 * FUNCTION - bf2253L_rio6_calculate_exposure -
 *
 * DESCRIPTION:
 *==========================================================================*/
static int32_t bf2253L_calculate_exposure(float real_gain,
  uint16_t line_count, sensor_exposure_info_t *exp_info)
{
  if (!exp_info) {
    return -1;
  }

  exp_info->reg_gain = bf2253L_real_to_register_gain(real_gain);
  float sensor_real_gain = bf2253L_register_to_real_gain(exp_info->reg_gain);
  exp_info->digital_gain = real_gain / sensor_real_gain;
  exp_info->line_count = line_count;
  return 0;
}

/*===========================================================================
 * FUNCTION - bf2253L_rio6_fill_exposure_array -
 *
 * DESCRIPTION:
 *==========================================================================*/
static int32_t bf2253L_fill_exposure_array(uint16_t gain, uint32_t line,
    uint32_t fl_lines, int32_t luma_avg, uint32_t fgain,
    struct msm_camera_i2c_reg_setting* reg_setting)
{
   int32_t rc = 0;

  uint16_t reg_count = 0;

  if (!reg_setting) {
    return -1;
  }
 
 	fl_lines = fl_lines & 0xffff;
	fl_lines = fl_lines < 1236?1236:fl_lines;
	fl_lines = fl_lines - 1236;
	 
//  ALOGE("#######gain = %d, line = %d, fl_lines = %d", gain, line, fl_lines);         
  if(line < 1) line = 1; /*anti color deviation on shot expoure*/

	reg_setting->reg_setting[reg_count].reg_addr =
	  sensor_lib_ptr.exp_gain_info->coarse_int_time_addr;
	reg_setting->reg_setting[reg_count].reg_data = (line & 0xFF00) >> 8;
	reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr =
    sensor_lib_ptr.exp_gain_info->coarse_int_time_addr + 1;
  reg_setting->reg_setting[reg_count].reg_data = (line & 0xFF);     
  reg_count++;

	reg_setting->reg_setting[reg_count].reg_addr =
	sensor_lib_ptr.exp_gain_info->global_gain_addr;
  reg_setting->reg_setting[reg_count].reg_data = (gain & 0xFF);
	reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0x08;
  reg_setting->reg_setting[reg_count].reg_data = (fl_lines & 0xFF00)>>8;
  reg_count++;

  reg_setting->reg_setting[reg_count].reg_addr = 0x07;
  reg_setting->reg_setting[reg_count].reg_data = (fl_lines & 0xFF);
  reg_count++;
  
    reg_setting->size = reg_count;
	reg_setting->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	reg_setting->data_type = MSM_CAMERA_I2C_BYTE_DATA;
	reg_setting->delay = 30;

	return rc;
}

static sensor_exposure_table_t bf2253L_expsoure_tbl = {
  .sensor_calculate_exposure = bf2253L_calculate_exposure,
  .sensor_fill_exposure_array = bf2253L_fill_exposure_array,
};

static sensor_lib_t sensor_lib_ptr = {
  /* sensor slave info */
  .sensor_slave_info = &sensor_slave_info,
  /* sensor init params */
  .sensor_init_params = &sensor_init_params,
  /* sensor output settings */
  .sensor_output = &sensor_output,
  /* sensor output register address*/
  .output_reg_addr = &output_reg_addr,
  /* sensor exposure gain register address */
  .exp_gain_info = &exp_gain_info,
  /* sensor aec info */
  .aec_info = &aec_info,
  /* sensor snapshot exposure wait frames info */
  .snapshot_exp_wait_frames = 1,
  /* number of frames to skip after start stream */
  .sensor_num_frame_skip = 3,
  /* number of frames to skip after start HDR stream */
  .sensor_num_HDR_frame_skip = 2,
  /* sensor exposure table size */
  .exposure_table_size = 5,
  /* sensor lens info */
  .default_lens_info = &default_lens_info,
  /* csi lane params */
  .csi_lane_params = &csi_lane_params,
  /* csi cid params */
  .csi_cid_params = bf2253L_cid_cfg,
  /* csi csid params array size */
  .csi_cid_params_size = ARRAY_SIZE(bf2253L_cid_cfg),
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
  /* resolution cf table */
  .sensor_res_cfg_table = &bf2253L_res_table,
  /* res settings */
  .res_settings_array = &res_settings_array,
  /* out info array */
  .out_info_array = &out_info_array,
  /* crop params array */
  .crop_params_array = &crop_params_array,
  /* csi params array */
  .csi_params_array = &csi_params_array,
  /* sensor port info array */
  .sensor_stream_info_array = &bf2253L_stream_info_array,
  /* exposure funtion table */
  .exposure_func_table = &bf2253L_expsoure_tbl,
  /* chromatix array */
  .chromatix_array = &bf2253L_lib_chromatix_array,
  .sync_exp_gain = 1,
};

/*===========================================================================
 * FUNCTION - bf2253L_rio6_open_lib -
 *
 * DESCRIPTION:
 *==========================================================================*/
void *bf2253L_open_lib(void)
{
  return &sensor_lib_ptr;
}
