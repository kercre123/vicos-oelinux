//**************************************************************************************************
// Copyright (c) 2017 Qualcomm Technologies, Inc.
// All Rights Reserved.
// Confidential and Proprietary - Qualcomm Technologies, Inc.
//**************************************************************************************************

#include <cstdio>
#include <cstddef>
#include "msmgbm_adreno_utils.h"
#include "msmgbm_platform_wrapper.h"
#include "gbm_priv.h"

#define INT(exp) static_cast<int>(exp)
#define UINT(exp) static_cast<unsigned int>(exp)
#define ALIGN(x, align) (((x) + ((align)-1)) & ~((align)-1))
#define ASTC_BLOCK_SIZE 16

bool g_ubwc_disable = false;

namespace msm_gbm {

extern "C" {

static platform_wrap *platform_wrap_ = NULL;

/**
 * Function to create a cpp object of platform wrapper class
 * @return   0 : success
 *           1 : failure
 */

bool platform_wrap_instnce(void) {
    if (platform_wrap_ == NULL) {
        platform_wrap_ = new platform_wrap();
        if (!platform_wrap_->init()) {
          return 1;
       }
    }
    return 0;
}

/**
 * Function to query the aligned width and hieght from the platform specific object
 * @return    : alignedw
 *            : alignedh
 */

void qry_aligned_wdth_hght(gbm_bufdesc *descriptor, unsigned int *alignedw,
                                                             unsigned int *alignedh) {
    platform_wrap_->get_aligned_wdth_hght(descriptor, alignedw,alignedh);
}


/**
 * Function to query size based on format from the platform wrapper
 * @return    : size
 *
 */
unsigned int qry_size(gbm_bufdesc *desc, unsigned int alignedw, unsigned int alignedh){
    return platform_wrap_->get_size(desc->Format, desc->Width, desc->Height, desc->Usage, alignedw,
                                    alignedh);
}

/**
 * Function to check whether the format is UBWC or not from the platform wrapper
 * @params    gbm format
 * @return    boolean 0 (non UBWC format)
 *                    1 (UBWC format)
 *
 */
bool is_valid_ubwc_format( int format) {
   return platform_wrap_->is_valid_ubwc_fmt(format);
}


/**
 * Function to check whether the format is uncompressed RGB or not from the platform wrapper
 * @params    gbm format
 * @return    boolean 0 (compressed RGB format)
 *                    1 (uncompressed RGB format)
 *
 */
bool is_valid_uncmprsd_rgb_format( int format) {
   return platform_wrap_->is_valid_uncmprsd_rgb_fmt(format);
}

/**
 * Function to return bytes per pixel for a given uncompressed RGB format
 * @params    uncompressed RGB gbm format
 * @return    bytes per pixel
 *
 */
uint32_t get_bpp_for_uncmprsd_rgb_format(int format) {
   return platform_wrap_->get_bpp_for_uncmprsd_rgb_fmt(format);
}


uint32_t get_rgb_ubwc_metabuffer_size(int width, int height, int bpp) {
   return platform_wrap_->get_rgb_ubwc_mb_size(width, height, bpp);
}

/**
 * Function to check whether the format is UBWC or not from the platform wrapper
 * @params    gbm format
 * @params    int prod_usage
 * @params    int cons_usage
 * @return    boolean 0 (non UBWC format)
 *                    1 (UBWC format)
 *
 */
bool is_ubwc_enabled(int format, int prod_usage, int cons_usage) {
   return platform_wrap_->is_ubwc_enbld(format, prod_usage, cons_usage);
}

/**
 * Function to delete platform wrapper object
 * @return  : None
 *
 */
void platform_wrap_deinstnce(void) {
    if(platform_wrap_) {
       delete platform_wrap_;
       platform_wrap_ = NULL;
    }
}

}

bool cpu_can_accss(int prod_usage, int cons_usage) {
  return cpu_can_wr(prod_usage);
}
bool cpu_can_wr(int prod_usage) {
  if (prod_usage & GBM_BO_USE_WRITE) {
    // Application intends to use CPU for rendering
    return true;
  }
  return false;
}

bool is_valid_cmprsd_rgb_fmt(int format) {
  switch (format) {
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:
    case GBM_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR:
    case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:
      return true;
    default:
      break;
  }

  return false;
}

platform_wrap::platform_wrap() {
}

bool platform_wrap::init() {
  adreno_helper_ = new adreno_mem_info();
  if (!adreno_helper_->init()) {
    return false;
  }

  gpu_support_macrotile = adreno_helper_->is_mcro_tiling_supported();
#if 0
  int supports_macrotile = 0;
  qdutils::querySDEInfo(qdutils::HAS_MACRO_TILE, &supports_macrotile);
  display_support_macrotile = !!supports_macrotile;
#endif
  LOG(LOG_DBG,"platform_wrap Success\n");

  return true;
}

platform_wrap::~platform_wrap() {
  if (adreno_helper_) {
    LOG(LOG_DBG,"Destroyer\n");
    delete adreno_helper_;
    adreno_helper_ = NULL;
  }
}

int platform_wrap::is_valid_rgb_fmt(int format){
    int is_supported;
    switch(format)
    {
        case GBM_FORMAT_RGB565:
        case GBM_FORMAT_RGB888:
        case GBM_FORMAT_RGBA8888:
        case GBM_FORMAT_RGBX8888:
        case GBM_FORMAT_XRGB8888:
        case GBM_FORMAT_XBGR8888:
        case GBM_FORMAT_ARGB8888:
        case GBM_FORMAT_ABGR8888:
        case GBM_FORMAT_BGR565:
        case GBM_FORMAT_ABGR2101010:
             is_supported = 1;
             break;
        default:
            is_supported = 0;
    }
    return is_supported;
}

uint32_t platform_wrap::get_bpp_for_uncmprsd_rgb_fmt(int format) {
  uint32_t bpp = 0;
  switch (format) {
    case GBM_FORMAT_RGBA8888:
    case GBM_FORMAT_RGBX8888:
    case GBM_FORMAT_BGRA8888:
    case GBM_FORMAT_BGRX8888:
    case GBM_FORMAT_XBGR8888:
    case GBM_FORMAT_ABGR8888:
    case GBM_FORMAT_ABGR2101010:
      bpp = 4;
      break;
    case GBM_FORMAT_RGB888:
      bpp = 3;
      break;
    case GBM_FORMAT_RGB565:
    case GBM_FORMAT_BGR565:
    case GBM_FORMAT_RGBA5551:
    case GBM_FORMAT_RGBA4444:
      bpp = 2;
      break;
    default:
      LOG(LOG_ERR," New format request\n");
      break;
  }
  return bpp;
}

/* Not currently supported formats
 case HAL_PIXEL_FORMAT_R_8:
 case HAL_PIXEL_FORMAT_RG_88:
*/
bool platform_wrap::is_valid_uncmprsd_rgb_fmt(int format) {
  switch (format) {
    case GBM_FORMAT_XRGB8888:
    case GBM_FORMAT_XBGR8888:
    case GBM_FORMAT_ARGB8888:
    case GBM_FORMAT_ABGR8888:
    case GBM_FORMAT_RGBA8888:
    case GBM_FORMAT_RGBX8888:
    case GBM_FORMAT_RGB888:
    case GBM_FORMAT_RGB565:
    case GBM_FORMAT_BGR565:
    case GBM_FORMAT_BGRA8888:
    case GBM_FORMAT_RGBA5551:
    case GBM_FORMAT_RGBA4444:
    case GBM_FORMAT_BGRX8888:
    case GBM_FORMAT_RGBA1010102:
    case GBM_FORMAT_ARGB2101010:
    case GBM_FORMAT_RGBX1010102:
    case GBM_FORMAT_XRGB2101010:
    case GBM_FORMAT_BGRA1010102:
    case GBM_FORMAT_ABGR2101010:
    case GBM_FORMAT_BGRX1010102:
    case GBM_FORMAT_XBGR2101010:
      return true;
    default:
      break;
  }

  return false;
}


// helper function
unsigned int platform_wrap::get_size(int format, int width, int height, int usage,
                                        int alignedw, int alignedh) {


   int prod_usage=usage;
   int cons_usage=usage;


    if (is_ubwc_enbld(format, prod_usage,cons_usage)) {
        return get_ubwc_size(width, height, format, alignedw, alignedh);
    }


    unsigned int size = 0;


    switch (format) {
        case GBM_FORMAT_RGBA8888:
        case GBM_FORMAT_RGBX8888:
        case GBM_FORMAT_BGRX8888:
        case GBM_FORMAT_BGRA8888:
        case GBM_FORMAT_XRGB8888:
        case GBM_FORMAT_XBGR8888:
        case GBM_FORMAT_ARGB8888:
        case GBM_FORMAT_ABGR8888:
        case GBM_FORMAT_RGBA1010102:
        case GBM_FORMAT_ARGB2101010:
        case GBM_FORMAT_RGBX1010102:
        case GBM_FORMAT_XRGB2101010:
        case GBM_FORMAT_BGRA1010102:
        case GBM_FORMAT_ABGR2101010:
        case GBM_FORMAT_BGRX1010102:
        case GBM_FORMAT_XBGR2101010:
            size = alignedw * alignedh * 4;
            break;
        case GBM_FORMAT_RGB888:
            size = alignedw * alignedh * 3;
            break;
        case GBM_FORMAT_RGB565:
        case GBM_FORMAT_BGR565:
        case GBM_FORMAT_RGBA5551:
        case GBM_FORMAT_RGBA4444:
        case GBM_FORMAT_RAW16:
            size = alignedw * alignedh * 2;
            break;
        case GBM_FORMAT_RAW10:
            size = ALIGN(alignedw * alignedh, 4096);
            break;
        case GBM_FORMAT_YV12:
            if ((format == GBM_FORMAT_YV12) && ((width&1) || (height&1))) {
                LOG(LOG_ERR," w or h is odd for the YV12 format\n");
                return 0;
            }
            size = alignedw*alignedh +
                    (ALIGN(alignedw/2, 16) * (alignedh/2))*2;
            size = ALIGN(size, (unsigned int)4096);
            break;
        case GBM_FORMAT_YCbCr_420_SP:
        case GBM_FORMAT_YCrCb_420_SP:
            size = ALIGN((alignedw*alignedh) + (alignedw* alignedh)/2 + 1, 4096);
            break;
        case GBM_FORMAT_YCbCr_422_SP:
        case GBM_FORMAT_YCrCb_422_SP:
        case GBM_FORMAT_YCbCr_422_I:
        case GBM_FORMAT_YCrCb_422_I:
            if(width & 1) {
                LOG(LOG_ERR," width is odd for the YUV422_SP format\n");
                return 0;
            }
            size = ALIGN(alignedw * alignedh * 2, 4096);
            break;
        case GBM_FORMAT_YCbCr_420_SP_VENUS:
        case GBM_FORMAT_NV12_ENCODEABLE:
        case GBM_FORMAT_NV12:
            size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height);
            LOG(LOG_INFO," VENUS_BUF_SIZE=%u, computed for Width=%u, Height=%u\n",
                                  size, width, height);
            break;
        case GBM_FORMAT_YCrCb_420_SP_VENUS:
            size = VENUS_BUFFER_SIZE(COLOR_FMT_NV21, width, height);
            break;
        case GBM_FORMAT_YCbCr_420_TP10_UBWC:
            // The macro returns the stride which is 4/3 times the width, hence * 3/4
            //*aligned_w = (VENUS_Y_STRIDE(COLOR_FMT_NV12_BPP10_UBWC, width) * 3) / 4;
            //*aligned_h = VENUS_Y_SCANLINES(COLOR_FMT_NV12_BPP10_UBWC, height);
            size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12_BPP10_UBWC, width, height);
            LOG(LOG_INFO," VENUS_BUF_SIZE=%u, computed for Width=%u, Height=%u\n",
                                  size, width, height);
            break;
        case GBM_FORMAT_P010:
            size = ALIGN((alignedw * alignedh * 2) + (alignedw * alignedh) + 1, 4096);
            break;
        case GBM_FORMAT_BLOB:
        case GBM_FORMAT_RAW_OPAQUE:
            if(height != 1) {
                LOG(LOG_ERR," Buffers with format HAL_PIXEL_FORMAT_BLOB \
                                        must have height==1\n");
                return 0;
            }
            size = width;
            break;
        case GBM_FORMAT_NV21_ZSL:
            size = ALIGN((alignedw*alignedh) + (alignedw* alignedh)/2, 4096);
            break;
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_4x4_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_5x4_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_5x5_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_6x5_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_6x6_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_8x5_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_8x6_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_8x8_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x5_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x6_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x8_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_10x10_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_12x10_KHR:
        case GBM_FORMAT_COMPRESSED_RGBA_ASTC_12x12_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:
        case GBM_FORMAT_COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:
            size = alignedw * alignedh * ASTC_BLOCK_SIZE;
            break;
        default:
            LOG(LOG_ERR," Unrecognized pixel format: 0x%x\n",format);
            return 0;
    }


    return size;
}

void platform_wrap::get_yuv_ubwc_wdth_hght(int width, int height, int format,
                                         unsigned int *aligned_w,unsigned int *aligned_h) {

  switch (format) {
    case GBM_FORMAT_NV12:
    case GBM_FORMAT_NV12_ENCODEABLE:
    case GBM_FORMAT_YCbCr_420_SP_VENUS:
    case GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      *aligned_w = VENUS_Y_STRIDE(COLOR_FMT_NV12_UBWC, width);
      *aligned_h = VENUS_Y_SCANLINES(COLOR_FMT_NV12_UBWC, height);
      break;
    case GBM_FORMAT_YCbCr_420_TP10_UBWC:
      // The macro returns the stride which is 4/3 times the width, hence * 3/4
      *aligned_w = (VENUS_Y_STRIDE(COLOR_FMT_NV12_BPP10_UBWC, width) * 3) / 4;
      *aligned_h = VENUS_Y_SCANLINES(COLOR_FMT_NV12_BPP10_UBWC, height);
      break;
    default:
      LOG(LOG_ERR," Unsupported pixel format: 0x%x\n",format);
      *aligned_w = 0;
      *aligned_h = 0;
      break;
  }

}

void platform_wrap::get_aligned_wdth_hght(gbm_bufdesc *descriptor, unsigned int *alignedw,
                                                   unsigned int *alignedh) {
  unsigned int width = descriptor->Width;
  unsigned int height = descriptor->Height;
  unsigned int format = descriptor->Format;
  unsigned int prod_usage = descriptor->Usage;
  unsigned int cons_usage = descriptor->Usage;
  unsigned int alignment = 32;
  bool ubwc_enabled = false;
  int tile = 0;

  LOG(LOG_DBG,"width=%d, height=%d,format=%d, usage=%d\n",width,height,
                                                     format,prod_usage);

  // Currently surface padding is only computed for RGB* surfaces.
  ubwc_enabled = is_ubwc_enbld(format, prod_usage, cons_usage);

  LOG(LOG_DBG,"ubwc_enabled=%d, tile=%d \n",ubwc_enabled,tile);

  tile = ubwc_enabled;

  LOG(LOG_DBG,"ubwc_enabled=%d, tile=%d \n",ubwc_enabled,tile);

  if (is_valid_uncmprsd_rgb_fmt(format))
    adreno_helper_->get_aligned_wdth_hght_uncmprsd_rgb_fmt(width, height, format, tile, alignedw, alignedh);

  if (ubwc_enabled && !is_valid_rgb_fmt(format))
    get_yuv_ubwc_wdth_hght(width, height, format, alignedw, alignedh);

  if (is_valid_cmprsd_rgb_fmt(format))
    adreno_helper_->get_aligned_wdth_hght_cmprsd_rgb_fmt(width, height, format, alignedw, alignedh);

  // Below should be only YUV family
  switch (format) {
    case GBM_FORMAT_YCrCb_420_SP:
    case GBM_FORMAT_YCbCr_420_SP:
      alignment = adreno_helper_->get_gpu_pxl_align();
      *alignedw = ALIGN(width, alignment);
      break;
    case GBM_FORMAT_RAW16:
      *alignedw = ALIGN(width, 16);
      break;
    case GBM_FORMAT_RAW10:
      *alignedw = ALIGN(width * 10 / 8, 16);
      break;
    case GBM_FORMAT_YV12:
    case GBM_FORMAT_YCbCr_422_SP:
    case GBM_FORMAT_YCrCb_422_SP:
    case GBM_FORMAT_YCbCr_422_I:
    case GBM_FORMAT_YCrCb_422_I:
    case GBM_FORMAT_P010:
      *alignedw = ALIGN(width, 16);
      *alignedh = height;
      break;
    case GBM_FORMAT_YCbCr_420_SP_VENUS:
    case GBM_FORMAT_NV12_ENCODEABLE:
      LOG(LOG_DBG,"@ YUV Format\n");
      *alignedw = INT(VENUS_Y_STRIDE(COLOR_FMT_NV12, width));
      *alignedh = INT(VENUS_Y_SCANLINES(COLOR_FMT_NV12, height));
      break;
    case GBM_FORMAT_NV12:
      if (ubwc_enabled) {
          *alignedw = INT(VENUS_Y_STRIDE(COLOR_FMT_NV12_UBWC, width));
          *alignedh = INT(VENUS_Y_SCANLINES(COLOR_FMT_NV12_UBWC, height));
      } else {
          *alignedw = INT(VENUS_Y_STRIDE(COLOR_FMT_NV12, width));
          *alignedh = INT(VENUS_Y_SCANLINES(COLOR_FMT_NV12, height));
      }
      break;
    case GBM_FORMAT_YCrCb_420_SP_VENUS:
      *alignedw = INT(VENUS_Y_STRIDE(COLOR_FMT_NV21, width));
      *alignedh = INT(VENUS_Y_SCANLINES(COLOR_FMT_NV21, height));
      break;
    case GBM_FORMAT_YCbCr_420_TP10_UBWC:
      // The macro returns the stride which is 4/3 times the width, hence * 3/4
      *alignedw = (VENUS_Y_STRIDE(COLOR_FMT_NV12_BPP10_UBWC, width) * 3) / 4;
      *alignedh = VENUS_Y_SCANLINES(COLOR_FMT_NV12_BPP10_UBWC, height);
      break;
    case GBM_FORMAT_BLOB:
    case GBM_FORMAT_RAW_OPAQUE:
      break;
    case GBM_FORMAT_NV21_ZSL:
      *alignedw = ALIGN(width, 64);
      *alignedh = ALIGN(height, 64);
      break;
    default:
      break;
  }

  LOG(LOG_DBG,"alignedw=%d, alignedh=%d \n",*alignedw,*alignedh);

}

// Explicitly defined UBWC formats Typically used for Video formats.
bool platform_wrap::is_valid_ubwc_fmt(int format) {
  switch (format) {
    case GBM_FORMAT_YCbCr_420_TP10_UBWC:
      return true;
    default:
      return false;
  }
}

bool platform_wrap::is_ubwc_support_enbld(int format) {
  // Existing HAL formats with UBWC support
  switch (format) {
    case GBM_FORMAT_BGR565:
    case GBM_FORMAT_XBGR8888:
    case GBM_FORMAT_ABGR8888:
    case GBM_FORMAT_ABGR2101010:
    case GBM_FORMAT_RGBA8888:
    case GBM_FORMAT_RGBX8888:
      return true;
    default:
      break;
  }

  return false;
}

bool platform_wrap::is_ubwc_enbld(int format, int prod_usage,
                              int cons_usage) {
    // debug flag to disable UBWC (useful for troubleshooting)
    if (g_ubwc_disable) {
        LOG(LOG_INFO,"UBWC feature is disabled\n");
        return false;
    }

    // Allow UBWC, if client is using an explicitly defined UBWC pixel format.
    if (is_valid_ubwc_fmt(format)) {
        return true;
    }

    // Allow UBWC for NV12, if client usage flags is set to GBM_BO_USAGE_UBWC_ALIGNED_QTI
    if ((prod_usage & GBM_BO_USAGE_UBWC_ALIGNED_QTI) && (format == GBM_FORMAT_NV12))
        return true;

    // Allow UBWC, if an OpenGL client sets UBWC usage flag and GPU plus MDP
    // support the format. OR if a non-OpenGL client like Rotator, sets UBWC
    // usage flag and MDP supports the format.
    if ((prod_usage & GBM_BO_USAGE_UBWC_ALIGNED_QTI) && is_ubwc_support_enbld(format)) {
        bool enable = true;

        // Query GPU for UBWC only if buffer is intended to be used by GPU.
        if ((cons_usage & GBM_BO_USAGE_HW_RENDERING_QTI) ||
            (prod_usage & GBM_BO_USAGE_HW_RENDERING_QTI)) {
            enable = adreno_helper_->is_ubwc_supported_gpu(format);
        }

        // Allow UBWC, only if CPU usage flags are not set
        if (enable && !(cpu_can_accss(prod_usage, cons_usage))) {
            return true;
        }
    }

    return false;
}
// TODO (user) : check for other formats with mcro-tile
bool platform_wrap::is_mcro_tile_enbld(int format, int prod_usage, int cons_usage) {
  bool tile_enabled = false;

  // Check whether GPU & MDSS supports MacroTiling feature
  if (!adreno_helper_->is_mcro_tiling_supported() || !display_support_macrotile) {
    return tile_enabled;
  }

  LOG(LOG_DBG," is_mcro_tiling_supported() success\n");

  // check the format
  switch (format) {
    case GBM_FORMAT_RGBA8888:
    case GBM_FORMAT_RGBX8888:
    case GBM_FORMAT_BGRA8888:
    case GBM_FORMAT_RGB565:
    case GBM_FORMAT_BGR565:
      if (!cpu_can_accss(prod_usage, cons_usage)) {
        // not touched by CPU
        tile_enabled = true;
      }
      break;
    default:
      break;
  }

  return tile_enabled;
}

void platform_wrap::get_rgb_ubwc_blk_size(uint32_t bpp, int *block_width, int *block_height) {
  *block_width = 0;
  *block_height = 0;

  switch (bpp) {
    case 2:
    case 4:
      *block_width = 16;
      *block_height = 4;
      break;
    case 8:
      *block_width = 8;
      *block_height = 4;
      break;
    case 16:
      *block_width = 4;
      *block_height = 4;
      break;
    default:
      LOG(LOG_ERR," Unsupported bpp: %d\n",bpp);
      break;
  }
}

unsigned int platform_wrap::get_rgb_ubwc_mb_size(int width, int height, uint32_t bpp) {
  unsigned int size = 0;
  int meta_width, meta_height;
  int block_width, block_height;

  get_rgb_ubwc_blk_size(bpp, &block_width, &block_height);
  if (!block_width || !block_height) {
    LOG(LOG_ERR," Unsupported bpp: %d\n",bpp);
    return size;
  }

  // Align meta buffer height to 16 blocks
  meta_height = ALIGN(((height + block_height - 1) / block_height), 16);

  // Align meta buffer width to 64 blocks
  meta_width = ALIGN(((width + block_width - 1) / block_width), 64);

  // Align meta buffer size to 4K
  size = (unsigned int)ALIGN((meta_width * meta_height), 4096);

  return size;
}

unsigned int platform_wrap::get_ubwc_size(int width, int height, int format, unsigned int alignedw,
                                                unsigned int alignedh) {
  unsigned int size = 0;
  uint32_t bpp = 0;
  switch (format) {
    case GBM_FORMAT_BGR565:
    case GBM_FORMAT_XBGR8888:
    case GBM_FORMAT_ABGR8888:
    case GBM_FORMAT_ABGR2101010:
      bpp = get_bpp_for_uncmprsd_rgb_fmt(format);
      size = alignedw * alignedh * bpp;
      size += get_rgb_ubwc_mb_size(width, height, bpp);
      break;
    case GBM_FORMAT_NV12_ENCODEABLE:
    case GBM_FORMAT_NV12:
    case GBM_FORMAT_YCbCr_420_SP_VENUS:
    case GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12_UBWC, width, height);
      break;
    case GBM_FORMAT_YCbCr_420_TP10_UBWC:
      size = VENUS_BUFFER_SIZE(COLOR_FMT_NV12_BPP10_UBWC, width, height);
      break;
    case GBM_FORMAT_YCbCr_420_P010_UBWC:
      size = VENUS_BUFFER_SIZE(COLOR_FMT_P010_UBWC, width, height);
      break;
    default:
      LOG(LOG_ERR," Unsupported pixel format: 0x%x\n",format);
      break;
  }

  return size;
}

}  // namespace msm_gbm
