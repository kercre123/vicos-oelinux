/*
* Copyright (c) 2014 - 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/rect.h>
#include <string>
#include <vector>
#include <algorithm>

#include "display_base.h"
#include "hw_info_interface.h"

#define __CLASS__ "DisplayBase"

namespace sdm {

// TODO(user): Have a single structure handle carries all the interface pointers and variables.
DisplayBase::DisplayBase(DisplayType display_type, DisplayEventHandler *event_handler,
                         HWDeviceType hw_device_type, BufferSyncHandler *buffer_sync_handler,
                         BufferAllocator *buffer_allocator, CompManager *comp_manager,
                         HWInfoInterface *hw_info_intf)
  : display_type_(display_type), event_handler_(event_handler), hw_device_type_(hw_device_type),
    buffer_sync_handler_(buffer_sync_handler), buffer_allocator_(buffer_allocator),
    comp_manager_(comp_manager), hw_info_intf_(hw_info_intf) {
}

DisplayError DisplayBase::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  hw_panel_info_ = HWPanelInfo();
  hw_intf_->GetHWPanelInfo(&hw_panel_info_);

  uint32_t active_index = 0;
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &display_attributes_);
  fb_config_ = display_attributes_;

  error = Debug::GetMixerResolution(&mixer_attributes_.width, &mixer_attributes_.height);
  if (error == kErrorNone) {
    hw_intf_->SetMixerAttributes(mixer_attributes_);
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes_);
  if (error != kErrorNone) {
    return error;
  }

  // Override x_pixels and y_pixels of frame buffer with mixer width and height
  fb_config_.x_pixels = mixer_attributes_.width;
  fb_config_.y_pixels = mixer_attributes_.height;

  HWScaleLutInfo lut_info = {};
  error = comp_manager_->GetScaleLutConfig(&lut_info);
  if (error == kErrorNone) {
    error = hw_intf_->SetScaleLutConfig(&lut_info);
  }

  if (error != kErrorNone) {
    goto CleanupOnError;
  }

  error = comp_manager_->RegisterDisplay(display_type_, display_attributes_, hw_panel_info_,
                                         mixer_attributes_, fb_config_, &display_comp_ctx_);
  if (error != kErrorNone) {
    goto CleanupOnError;
  }

  if (hw_info_intf_) {
    HWResourceInfo hw_resource_info = HWResourceInfo();
    hw_info_intf_->GetHWResourceInfo(&hw_resource_info);
    auto max_mixer_stages = hw_resource_info.num_blending_stages;
    int property_value = Debug::GetMaxPipesPerMixer(display_type_);
    if (property_value >= 0) {
      max_mixer_stages = std::min(UINT32(property_value), hw_resource_info.num_blending_stages);
    }
    DisplayBase::SetMaxMixerStages(max_mixer_stages);
  }

  color_mgr_ = ColorManagerProxy::CreateColorManagerProxy(display_type_, hw_intf_,
                               display_attributes_, hw_panel_info_);
  if (!color_mgr_) {
    DLOGW("Unable to create ColorManagerProxy for display = %d", display_type_);
  } else if (InitializeColorModes() != kErrorNone) {
    DLOGW("InitColorModes failed for display = %d", display_type_);
  }

  Debug::Get()->GetProperty("sdm.disable_hdr_lut_gen", &disable_hdr_lut_gen_);

  return kErrorNone;

CleanupOnError:
  if (display_comp_ctx_) {
    comp_manager_->UnregisterDisplay(display_comp_ctx_);
  }

  return error;
}

DisplayError DisplayBase::Deinit() {
  {  // Scope for lock
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    color_modes_.clear();
    color_mode_map_.clear();

    if (color_mgr_) {
      delete color_mgr_;
      color_mgr_ = NULL;
    }

    comp_manager_->UnregisterDisplay(display_comp_ctx_);
  }
  HWEventsInterface::Destroy(hw_events_intf_);
  HWInterface::Destroy(hw_intf_);

  return kErrorNone;
}

DisplayError DisplayBase::BuildLayerStackStats(LayerStack *layer_stack) {
  std::vector<Layer *> &layers = layer_stack->layers;
  HWLayersInfo &hw_layers_info = hw_layers_.info;

  hw_layers_info.stack = layer_stack;

  for (auto &layer : layers) {
    if (layer->composition == kCompositionGPUTarget) {
      hw_layers_info.gpu_target_index = hw_layers_info.app_layer_count;
      break;
    }
    hw_layers_info.app_layer_count++;
  }

  DLOGV_IF(kTagNone, "LayerStack layer_count: %d, app_layer_count: %d, gpu_target_index: %d, "
           "display type: %d", layers.size(), hw_layers_info.app_layer_count,
           hw_layers_info.gpu_target_index, display_type_);

  if (!hw_layers_info.app_layer_count) {
    DLOGW("Layer count is zero");
    return kErrorNoAppLayers;
  }

  if (hw_layers_info.gpu_target_index) {
    return ValidateGPUTargetParams();
  }

  return kErrorNone;
}

DisplayError DisplayBase::ValidateGPUTargetParams() {
  HWLayersInfo &hw_layers_info = hw_layers_.info;
  Layer *gpu_target_layer = hw_layers_info.stack->layers.at(hw_layers_info.gpu_target_index);

  if (!IsValid(gpu_target_layer->src_rect)) {
    DLOGE("Invalid src rect for GPU target layer");
    return kErrorParameters;
  }

  if (!IsValid(gpu_target_layer->dst_rect)) {
    DLOGE("Invalid dst rect for GPU target layer");
    return kErrorParameters;
  }

  float layer_mixer_width = FLOAT(mixer_attributes_.width);
  float layer_mixer_height = FLOAT(mixer_attributes_.height);
  float fb_width = FLOAT(fb_config_.x_pixels);
  float fb_height = FLOAT(fb_config_.y_pixels);
  LayerRect src_domain = (LayerRect){0.0f, 0.0f, fb_width, fb_height};
  LayerRect dst_domain = (LayerRect){0.0f, 0.0f, layer_mixer_width, layer_mixer_height};
  LayerRect out_rect = gpu_target_layer->dst_rect;

  MapRect(src_domain, dst_domain, gpu_target_layer->dst_rect, &out_rect);
  Normalize(1, 1, &out_rect);

  auto gpu_target_layer_dst_xpixels = out_rect.right - out_rect.left;
  auto gpu_target_layer_dst_ypixels = out_rect.bottom - out_rect.top;

  if (gpu_target_layer_dst_xpixels > mixer_attributes_.width ||
    gpu_target_layer_dst_ypixels > mixer_attributes_.height) {
    DLOGE("GPU target layer dst rect is not with in limits gpu wxh %fx%f, mixer wxh %dx%d",
                  gpu_target_layer_dst_xpixels, gpu_target_layer_dst_ypixels,
                  mixer_attributes_.width, mixer_attributes_.height);
    return kErrorParameters;
  }

  return kErrorNone;
}

DisplayError DisplayBase::Prepare(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (!active_) {
    return kErrorPermission;
  }

  if (!layer_stack) {
    return kErrorParameters;
  }

  error = BuildLayerStackStats(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  error = HandleHDR(layer_stack);
  if (error != kErrorNone) {
    DLOGW("HandleHDR failed");
    return error;
  }

  if (color_mgr_ && color_mgr_->NeedsPartialUpdateDisable()) {
    DisablePartialUpdateOneFrame();
  }

  if (partial_update_control_ == false || disable_pu_one_frame_) {
    comp_manager_->ControlPartialUpdate(display_comp_ctx_, false /* enable */);
    disable_pu_one_frame_ = false;
  }

  comp_manager_->PrePrepare(display_comp_ctx_, &hw_layers_);
  while (true) {
    error = comp_manager_->Prepare(display_comp_ctx_, &hw_layers_);
    if (error != kErrorNone) {
      break;
    }

    error = hw_intf_->Validate(&hw_layers_);
    if (error == kErrorNone) {
      // Strategy is successful now, wait for Commit().
      pending_commit_ = true;
      break;
    }
    if (error == kErrorShutDown) {
      comp_manager_->PostPrepare(display_comp_ctx_, &hw_layers_);
      return error;
    }
  }

  comp_manager_->PostPrepare(display_comp_ctx_, &hw_layers_);

  return error;
}

DisplayError DisplayBase::Commit(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (!active_) {
    pending_commit_ = false;
    return kErrorPermission;
  }

  if (!layer_stack) {
    return kErrorParameters;
  }

  if (!pending_commit_) {
    DLOGE("Commit: Corresponding Prepare() is not called for display = %d", display_type_);
    return kErrorUndefined;
  }

  pending_commit_ = false;

  // Layer stack attributes has changed, need to Reconfigure, currently in use for Hybrid Comp
  if (layer_stack->flags.attributes_changed) {
    error = comp_manager_->ReConfigure(display_comp_ctx_, &hw_layers_);
    if (error != kErrorNone) {
      return error;
    }

    error = hw_intf_->Validate(&hw_layers_);
    if (error != kErrorNone) {
        return error;
    }
  }

  CommitLayerParams(layer_stack);

  if (comp_manager_->Commit(display_comp_ctx_, &hw_layers_)) {
    if (error != kErrorNone) {
      return error;
    }
  }

  // check if feature list cache is dirty and pending.
  // If dirty, need program to hardware blocks.
  if (color_mgr_)
    error = color_mgr_->Commit();
  if (error != kErrorNone) {  // won't affect this execution path.
    DLOGW("ColorManager::Commit(...) isn't working");
  }

  error = hw_intf_->Commit(&hw_layers_);
  if (error != kErrorNone) {
    return error;
  }

  PostCommitLayerParams(layer_stack);

  if (partial_update_control_) {
    comp_manager_->ControlPartialUpdate(display_comp_ctx_, true /* enable */);
  }

  error = comp_manager_->PostCommit(display_comp_ctx_, &hw_layers_);
  if (error != kErrorNone) {
    return error;
  }

  return kErrorNone;
}

DisplayError DisplayBase::Flush() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (!active_) {
    return kErrorPermission;
  }
  hw_layers_.info.hw_layers.clear();
  error = hw_intf_->Flush();
  if (error == kErrorNone) {
    comp_manager_->Purge(display_comp_ctx_);
    pending_commit_ = false;
  } else {
    DLOGW("Unable to flush display = %d", display_type_);
  }

  return error;
}

DisplayError DisplayBase::GetDisplayState(DisplayState *state) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!state) {
    return kErrorParameters;
  }

  *state = state_;
  return kErrorNone;
}

DisplayError DisplayBase::GetNumVariableInfoConfigs(uint32_t *count) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->GetNumDisplayAttributes(count);
}

DisplayError DisplayBase::GetConfig(uint32_t index, DisplayConfigVariableInfo *variable_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  HWDisplayAttributes attrib;
  if (hw_intf_->GetDisplayAttributes(index, &attrib) == kErrorNone) {
    *variable_info = attrib;
    return kErrorNone;
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  fixed_info->is_cmdmode = (hw_panel_info_.mode == kModeCommand);

  HWResourceInfo hw_resource_info = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_info);
  // hdr can be supported by display when target and panel supports HDR.
  fixed_info->hdr_supported = (hw_resource_info.has_hdr && hw_panel_info_.hdr_enabled);
  // Populate luminance values only if hdr will be supported on that display
  fixed_info->max_luminance = fixed_info->hdr_supported ? hw_panel_info_.peak_luminance: 0;
  fixed_info->average_luminance = fixed_info->hdr_supported ? hw_panel_info_.average_luminance : 0;
  fixed_info->min_luminance = fixed_info->hdr_supported ?  hw_panel_info_.blackness_level: 0;
  fixed_info->hdr_eotf = hw_panel_info_.hdr_eotf;
  fixed_info->hdr_metadata_type_one = hw_panel_info_.hdr_metadata_type_one;

  return kErrorNone;
}

DisplayError DisplayBase::GetActiveConfig(uint32_t *index) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->GetActiveConfig(index);
}

DisplayError DisplayBase::GetVSyncState(bool *enabled) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!enabled) {
    return kErrorParameters;
  }

  *enabled = vsync_enable_;

  return kErrorNone;
}

DisplayError DisplayBase::SetDisplayState(DisplayState state) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  bool active = false;

  DLOGI("Set state = %d, display %d", state, display_type_);

  if (state == state_) {
    DLOGI("Same state transition is requested.");
    return kErrorNone;
  }

  switch (state) {
  case kStateOff:
    hw_layers_.info.hw_layers.clear();
    error = hw_intf_->Flush();
    if (error == kErrorNone) {
      error = hw_intf_->PowerOff();
    }
    break;

  case kStateOn:
    error = hw_intf_->PowerOn();
    if (error != kErrorNone) {
      return error;
    }

    error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes_,
                                              hw_panel_info_, mixer_attributes_, fb_config_);
    if (error != kErrorNone) {
      return error;
    }

    active = true;
    break;

  case kStateDoze:
    error = hw_intf_->Doze();
    active = true;
    break;

  case kStateDozeSuspend:
    error = hw_intf_->DozeSuspend();
    if (display_type_ != kPrimary) {
      active = true;
    }

    break;

  case kStateStandby:
    error = hw_intf_->Standby();
    break;

  default:
    DLOGE("Spurious state = %d transition requested.", state);
    break;
  }

  if (error == kErrorNone) {
    active_ = active;
    state_ = state;
    comp_manager_->SetDisplayState(display_comp_ctx_, state, display_type_);
  }

  return error;
}

DisplayError DisplayBase::SetActiveConfig(uint32_t index) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t active_index = 0;

  hw_intf_->GetActiveConfig(&active_index);

  if (active_index == index) {
    return kErrorNone;
  }

  error = hw_intf_->SetDisplayAttributes(index);
  if (error != kErrorNone) {
    return error;
  }

  return ReconfigureDisplay();
}

DisplayError DisplayBase::SetMaxMixerStages(uint32_t max_mixer_stages) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  error = comp_manager_->SetMaxMixerStages(display_comp_ctx_, max_mixer_stages);

  if (error == kErrorNone) {
    max_mixer_stages_ = max_mixer_stages;
  }

  return error;
}

void DisplayBase::AppendDump(char *buffer, uint32_t length) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  HWDisplayAttributes attrib;
  uint32_t active_index = 0;
  uint32_t num_modes = 0;
  hw_intf_->GetNumDisplayAttributes(&num_modes);
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &attrib);

  DumpImpl::AppendString(buffer, length, "\n-----------------------");
  DumpImpl::AppendString(buffer, length, "\ndevice type: %u", display_type_);
  DumpImpl::AppendString(buffer, length, "\nstate: %u, vsync on: %u, max. mixer stages: %u",
                         state_, INT(vsync_enable_), max_mixer_stages_);
  DumpImpl::AppendString(buffer, length, "\nnum configs: %u, active config index: %u",
                         num_modes, active_index);

  DisplayConfigVariableInfo &info = attrib;

  uint32_t num_hw_layers = 0;
  if (hw_layers_.info.stack) {
    num_hw_layers = UINT32(hw_layers_.info.hw_layers.size());
  }

  if (num_hw_layers == 0) {
    DumpImpl::AppendString(buffer, length, "\nNo hardware layers programmed");
    return;
  }

  LayerBuffer *out_buffer = hw_layers_.info.stack->output_buffer;
  if (out_buffer) {
    DumpImpl::AppendString(buffer, length, "\nres:%u x %u format: %s", out_buffer->width,
                           out_buffer->height, GetFormatString(out_buffer->format));
  } else {
    DumpImpl::AppendString(buffer, length, "\nres:%u x %u, dpi:%.2f x %.2f, fps:%u,"
                           "vsync period: %u", info.x_pixels, info.y_pixels, info.x_dpi,
                           info.y_dpi, info.fps, info.vsync_period_ns);
  }

  DumpImpl::AppendString(buffer, length, "\n");

  HWLayersInfo &layer_info = hw_layers_.info;

  for (uint32_t i = 0; i < layer_info.left_frame_roi.size(); i++) {
    LayerRect &l_roi = layer_info.left_frame_roi.at(i);
    LayerRect &r_roi = layer_info.right_frame_roi.at(i);

    DumpImpl::AppendString(buffer, length, "\nROI%d(L T R B) : LEFT(%d %d %d %d)", i,
                           INT(l_roi.left), INT(l_roi.top), INT(l_roi.right), INT(l_roi.bottom));
    if (IsValid(r_roi)) {
      DumpImpl::AppendString(buffer, length, ", RIGHT(%d %d %d %d)", INT(r_roi.left),
                             INT(r_roi.top), INT(r_roi.right), INT(r_roi.bottom));
    }
  }

  LayerRect &fb_roi = layer_info.partial_fb_roi;
  if (IsValid(fb_roi)) {
    DumpImpl::AppendString(buffer, length, "\nPartial FB ROI(L T R B) : (%d %d %d %d)",
                          INT(fb_roi.left), INT(fb_roi.top), INT(fb_roi.right), INT(fb_roi.bottom));
  }

  const char *header  = "\n| Idx |  Comp Type  |  Split | WB |  Pipe  |    W x H    |          Format          |  Src Rect (L T R B) |  Dst Rect (L T R B) |  Z |    Flags   | Deci(HxV) | CS | Rng |";  //NOLINT
  const char *newline = "\n|-----|-------------|--------|----|--------|-------------|--------------------------|---------------------|---------------------|----|------------|-----------|----|-----|";  //NOLINT
  const char *format  = "\n| %3s | %11s "     "| %6s " "| %2s | 0x%04x | %4d x %4d | %24s "                  "| %4d %4d %4d %4d "  "| %4d %4d %4d %4d "  "| %2s | %10s "   "| %9s | %2s | %3s |";  //NOLINT

  DumpImpl::AppendString(buffer, length, "\n");
  DumpImpl::AppendString(buffer, length, newline);
  DumpImpl::AppendString(buffer, length, header);
  DumpImpl::AppendString(buffer, length, newline);

  for (uint32_t i = 0; i < num_hw_layers; i++) {
    uint32_t layer_index = hw_layers_.info.index[i];
    // sdm-layer from client layer stack
    Layer *sdm_layer = hw_layers_.info.stack->layers.at(layer_index);
    // hw-layer from hw layers info
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);
    LayerBuffer *input_buffer = &hw_layer.input_buffer;
    HWLayerConfig &layer_config = hw_layers_.config[i];
    HWRotatorSession &hw_rotator_session = layer_config.hw_rotator_session;

    char idx[8] = { 0 };
    const char *comp_type = GetName(sdm_layer->composition);
    const char *buffer_format = GetFormatString(input_buffer->format);
    const char *rotate_split[2] = { "Rot-1", "Rot-2" };
    const char *comp_split[2] = { "Comp-1", "Comp-2" };

    snprintf(idx, sizeof(idx), "%d", layer_index);

    for (uint32_t count = 0; count < hw_rotator_session.hw_block_count; count++) {
      char writeback_id[8] = { 0 };
      HWRotateInfo &rotate = hw_rotator_session.hw_rotate_info[count];
      LayerRect &src_roi = rotate.src_roi;
      LayerRect &dst_roi = rotate.dst_roi;

      snprintf(writeback_id, sizeof(writeback_id), "%d", rotate.writeback_id);

      DumpImpl::AppendString(buffer, length, format, idx, comp_type, rotate_split[count],
                             writeback_id, rotate.pipe_id, input_buffer->width,
                             input_buffer->height, buffer_format, INT(src_roi.left),
                             INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
                             INT(dst_roi.left), INT(dst_roi.top), INT(dst_roi.right),
                             INT(dst_roi.bottom), "-", "-    ", "-    ", "-", "-");

      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }

    if (hw_rotator_session.hw_block_count > 0) {
      input_buffer = &hw_rotator_session.output_buffer;
      buffer_format = GetFormatString(input_buffer->format);
    }

    for (uint32_t count = 0; count < 2; count++) {
      char decimation[16] = { 0 };
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      char color_primary[8] = { 0 };
      char range[8] = { 0 };

      HWPipeInfo &pipe = (count == 0) ? layer_config.left_pipe : layer_config.right_pipe;

      if (!pipe.valid) {
        continue;
      }

      LayerRect &src_roi = pipe.src_roi;
      LayerRect &dst_roi = pipe.dst_roi;

      snprintf(z_order, sizeof(z_order), "%d", pipe.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", hw_layer.flags.flags);
      snprintf(decimation, sizeof(decimation), "%3d x %3d", pipe.horizontal_decimation,
               pipe.vertical_decimation);
      ColorMetaData &color_metadata = hw_layer.input_buffer.color_metadata;
      snprintf(color_primary, sizeof(color_primary), "%d", color_metadata.colorPrimaries);
      snprintf(range, sizeof(range), "%d", color_metadata.range);

      DumpImpl::AppendString(buffer, length, format, idx, comp_type, comp_split[count],
                             "-", pipe.pipe_id, input_buffer->width, input_buffer->height,
                             buffer_format, INT(src_roi.left), INT(src_roi.top),
                             INT(src_roi.right), INT(src_roi.bottom), INT(dst_roi.left),
                             INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
                             z_order, flags, decimation, color_primary, range);

      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }

    DumpImpl::AppendString(buffer, length, newline);
  }
}

const char * DisplayBase::GetName(const LayerComposition &composition) {
  switch (composition) {
  case kCompositionGPU:         return "GPU";
  case kCompositionSDE:         return "SDE";
  case kCompositionHWCursor:    return "CURSOR";
  case kCompositionHybrid:      return "HYBRID";
  case kCompositionBlit:        return "BLIT";
  case kCompositionGPUTarget:   return "GPU_TARGET";
  case kCompositionBlitTarget:  return "BLIT_TARGET";
  default:                      return "UNKNOWN";
  }
}

DisplayError DisplayBase::ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                               PPDisplayAPIPayload *out_payload,
                                               PPPendingParams *pending_action) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (color_mgr_)
    return color_mgr_->ColorSVCRequestRoute(in_payload, out_payload, pending_action);
  else
    return kErrorParameters;
}

DisplayError DisplayBase::GetColorModeCount(uint32_t *mode_count) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_count) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DLOGV_IF(kTagQDCM, "Number of modes from color manager = %d", num_color_modes_);
  *mode_count = num_color_modes_;

  return kErrorNone;
}

DisplayError DisplayBase::GetColorModes(uint32_t *mode_count,
                                        std::vector<std::string> *color_modes) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_count || !color_modes) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  for (uint32_t i = 0; i < num_color_modes_; i++) {
    DLOGV_IF(kTagQDCM, "Color Mode[%d]: Name = %s mode_id = %d", i, color_modes_[i].name,
             color_modes_[i].id);
    color_modes->at(i) = color_modes_[i].name;
  }

  return kErrorNone;
}

DisplayError DisplayBase::SetColorMode(const std::string &color_mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DisplayError error = kErrorNone;
  // Set client requests when not in HDR Mode or lut generation is disabled
  if (disable_hdr_lut_gen_ || !hdr_playback_mode_) {
    error = SetColorModeInternal(color_mode);
    if (error != kErrorNone) {
      return error;
    }
  }
  // Store the new color mode request by client
  current_color_mode_ = color_mode;

  return error;
}

DisplayError DisplayBase::SetColorModeInternal(const std::string &color_mode) {
  DLOGV_IF(kTagQDCM, "Color Mode = %s", color_mode.c_str());

  ColorModeMap::iterator it = color_mode_map_.find(color_mode);
  if (it == color_mode_map_.end()) {
    DLOGE("Failed: Unknown Mode : %s", color_mode.c_str());
    return kErrorNotSupported;
  }

  SDEDisplayMode *sde_display_mode = it->second;

  DLOGV_IF(kTagQDCM, "Color Mode Name = %s corresponding mode_id = %d", sde_display_mode->name,
           sde_display_mode->id);
  DisplayError error = kErrorNone;
  error = color_mgr_->ColorMgrSetMode(sde_display_mode->id);
  if (error != kErrorNone) {
    DLOGE("Failed for mode id = %d", sde_display_mode->id);
    return error;
  }

  return error;
}

DisplayError DisplayBase::SetColorTransform(const uint32_t length, const double *color_transform) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  if (!color_transform) {
    return kErrorParameters;
  }

  return color_mgr_->ColorMgrSetColorTransform(length, color_transform);
}

DisplayError DisplayBase::ApplyDefaultDisplayMode() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (color_mgr_)
    return color_mgr_->ApplyDefaultDisplayMode();
  else
    return kErrorParameters;
}

DisplayError DisplayBase::SetCursorPosition(int x, int y) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (state_ != kStateOn) {
    return kErrorNotSupported;
  }

  DisplayError error = comp_manager_->ValidateCursorPosition(display_comp_ctx_, &hw_layers_, x, y);
  if (error == kErrorNone) {
    return hw_intf_->SetCursorPosition(&hw_layers_, x, y);
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                              uint32_t *max_refresh_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  // The min and max refresh rates will be same when the HWPanelInfo does not contain valid rates.
  // Usually for secondary displays, command mode panels
  HWDisplayAttributes display_attributes;
  uint32_t active_index = 0;
  hw_intf_->GetActiveConfig(&active_index);
  DisplayError error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error) {
    return error;
  }

  *min_refresh_rate = display_attributes.fps;
  *max_refresh_rate = display_attributes.fps;

  return error;
}

DisplayError DisplayBase::SetVSyncState(bool enable) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  if (vsync_enable_ != enable) {
    error = hw_intf_->SetVSyncState(enable);
    if (error == kErrorNotSupported) {
      error = hw_events_intf_->SetEventState(HWEvent::VSYNC, enable);
    }
    if (error == kErrorNone) {
      vsync_enable_ = pflip_enable_ = enable;
    }
  }

  return error;
}

DisplayError DisplayBase::ReconfigureDisplay() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  HWDisplayAttributes display_attributes;
  HWMixerAttributes mixer_attributes;
  HWPanelInfo hw_panel_info;
  uint32_t active_index = 0;

  error = hw_intf_->GetActiveConfig(&active_index);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetHWPanelInfo(&hw_panel_info);
  if (error != kErrorNone) {
    return error;
  }

  if (display_attributes == display_attributes_ && mixer_attributes == mixer_attributes_ &&
      hw_panel_info == hw_panel_info_) {
    return kErrorNone;
  }

  error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes, hw_panel_info,
                                            mixer_attributes, fb_config_);
  if (error != kErrorNone) {
    return error;
  }

  if (mixer_attributes != mixer_attributes_) {
    DisablePartialUpdateOneFrame();
  }

  display_attributes_ = display_attributes;
  mixer_attributes_ = mixer_attributes;
  hw_panel_info_ = hw_panel_info;

  return kErrorNone;
}

DisplayError DisplayBase::SetMixerResolution(uint32_t width, uint32_t height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = ReconfigureMixer(width, height);
  if (error != kErrorNone) {
    return error;
  }

  req_mixer_width_ = width;
  req_mixer_height_ = height;

  return kErrorNone;
}

DisplayError DisplayBase::GetMixerResolution(uint32_t *width, uint32_t *height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!width || !height) {
    return kErrorParameters;
  }

  *width = mixer_attributes_.width;
  *height = mixer_attributes_.height;

  return kErrorNone;
}

DisplayError DisplayBase::ReconfigureMixer(uint32_t width, uint32_t height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (!width || !height) {
    return kErrorParameters;
  }

  HWMixerAttributes mixer_attributes;
  mixer_attributes.width = width;
  mixer_attributes.height = height;

  error = hw_intf_->SetMixerAttributes(mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  return ReconfigureDisplay();
}

bool DisplayBase::NeedsDownScale(const LayerRect &src_rect, const LayerRect &dst_rect,
                                 bool needs_rotation) {
  float src_width = FLOAT(src_rect.right - src_rect.left);
  float src_height = FLOAT(src_rect.bottom - src_rect.top);
  float dst_width = FLOAT(dst_rect.right - dst_rect.left);
  float dst_height = FLOAT(dst_rect.bottom - dst_rect.top);

  if (needs_rotation) {
    std::swap(src_width, src_height);
  }

  if ((src_width > dst_width) || (src_height > dst_height)) {
    return true;
  }

  return false;
}

bool DisplayBase::NeedsMixerReconfiguration(LayerStack *layer_stack, uint32_t *new_mixer_width,
                                            uint32_t *new_mixer_height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  uint32_t layer_count = UINT32(layer_stack->layers.size());

  uint32_t fb_width  = fb_config_.x_pixels;
  uint32_t fb_height  = fb_config_.y_pixels;
  uint32_t fb_area = fb_width * fb_height;
  LayerRect fb_rect = (LayerRect) {0.0f, 0.0f, FLOAT(fb_width), FLOAT(fb_height)};
  uint32_t mixer_width = mixer_attributes_.width;
  uint32_t mixer_height = mixer_attributes_.height;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  RectOrientation fb_orientation = GetOrientation(fb_rect);
  uint32_t max_layer_area = 0;
  uint32_t max_area_layer_index = 0;
  std::vector<Layer *> layers = layer_stack->layers;
  uint32_t align_x = display_attributes_.is_device_split ? 4 : 2;
  uint32_t align_y = 2;

  if (req_mixer_width_ && req_mixer_height_) {
    *new_mixer_width = req_mixer_width_;
    *new_mixer_height = req_mixer_height_;

    return (req_mixer_width_ != mixer_width || req_mixer_height_ != mixer_height);
  }

  for (uint32_t i = 0; i < layer_count; i++) {
    Layer *layer = layers.at(i);

    uint32_t layer_width = UINT32(layer->src_rect.right - layer->src_rect.left);
    uint32_t layer_height = UINT32(layer->src_rect.bottom - layer->src_rect.top);
    uint32_t layer_area = layer_width * layer_height;

    if (layer_area > max_layer_area) {
      max_layer_area = layer_area;
      max_area_layer_index = i;
    }
  }

  // TODO(user): Mark layer which needs downscaling on GPU fallback as priority layer and use MDP
  // for composition to avoid quality mismatch between GPU and MDP switch(idle timeout usecase).
  if (max_layer_area >= fb_area) {
    Layer *layer = layers.at(max_area_layer_index);
    bool needs_rotation = (layer->transform.rotation == 90.0f);

    uint32_t layer_width = UINT32(layer->src_rect.right - layer->src_rect.left);
    uint32_t layer_height = UINT32(layer->src_rect.bottom - layer->src_rect.top);
    LayerRect layer_dst_rect = {};

    RectOrientation layer_orientation = GetOrientation(layer->src_rect);
    if (layer_orientation != kOrientationUnknown &&
        fb_orientation != kOrientationUnknown) {
      if (layer_orientation != fb_orientation) {
        std::swap(layer_width, layer_height);
      }
    }

    // Align the width and height according to fb's aspect ratio
    *new_mixer_width = FloorToMultipleOf(UINT32((FLOAT(fb_width) / FLOAT(fb_height)) *
                                         layer_height), align_x);
    *new_mixer_height = FloorToMultipleOf(layer_height, align_y);

    LayerRect dst_domain = {0.0f, 0.0f, FLOAT(*new_mixer_width), FLOAT(*new_mixer_height)};

    MapRect(fb_rect, dst_domain, layer->dst_rect, &layer_dst_rect);
    if (NeedsDownScale(layer->src_rect, layer_dst_rect, needs_rotation)) {
      *new_mixer_width = display_width;
      *new_mixer_height = display_height;
    }

    return true;
  }

  return false;
}

DisplayError DisplayBase::SetFrameBufferConfig(const DisplayConfigVariableInfo &variable_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  uint32_t width = variable_info.x_pixels;
  uint32_t height = variable_info.y_pixels;

  if (width == 0 || height == 0) {
    DLOGE("Unsupported resolution: (%dx%d)", width, height);
    return kErrorParameters;
  }

  // Create rects to represent the new source and destination crops
  LayerRect crop = LayerRect(0, 0, FLOAT(width), FLOAT(height));
  LayerRect dst = LayerRect(0, 0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height));
  // Set rotate90 to false since this is taken care of during regular composition.
  bool rotate90 = false;

  DisplayError error = comp_manager_->ValidateScaling(crop, dst, rotate90);
  if (error != kErrorNone) {
    DLOGE("Unsupported resolution: (%dx%d)", width, height);
    return kErrorParameters;
  }

  error =  comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes_, hw_panel_info_,
                                             mixer_attributes_, variable_info);
  if (error != kErrorNone) {
    return error;
  }

  fb_config_.x_pixels = width;
  fb_config_.y_pixels = height;

  DLOGI("New framebuffer resolution (%dx%d)", fb_config_.x_pixels, fb_config_.y_pixels);

  return kErrorNone;
}

DisplayError DisplayBase::GetFrameBufferConfig(DisplayConfigVariableInfo *variable_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!variable_info) {
    return kErrorParameters;
  }

  *variable_info = fb_config_;

  return kErrorNone;
}

DisplayError DisplayBase::SetDetailEnhancerData(const DisplayDetailEnhancerData &de_data) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = comp_manager_->SetDetailEnhancerData(display_comp_ctx_, de_data);
  if (error != kErrorNone) {
    return error;
  }

  DisablePartialUpdateOneFrame();

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayPort(DisplayPort *port) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!port) {
    return kErrorParameters;
  }

  *port = hw_panel_info_.port;

  return kErrorNone;
}

bool DisplayBase::IsPrimaryDisplay() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  return hw_panel_info_.is_primary_panel;
}

DisplayError DisplayBase::SetCompositionState(LayerComposition composition_type, bool enable) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  return comp_manager_->SetCompositionState(display_comp_ctx_, composition_type, enable);
}

void DisplayBase::CommitLayerParams(LayerStack *layer_stack) {
  // Copy the acquire fence from clients layers  to HWLayers
  uint32_t hw_layers_count = UINT32(hw_layers_.info.hw_layers.size());

  for (uint32_t i = 0; i < hw_layers_count; i++) {
    Layer *sdm_layer = layer_stack->layers.at(hw_layers_.info.index[i]);
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);

    hw_layer.input_buffer.planes[0].fd = sdm_layer->input_buffer.planes[0].fd;
    hw_layer.input_buffer.planes[0].offset = sdm_layer->input_buffer.planes[0].offset;
    hw_layer.input_buffer.planes[0].stride = sdm_layer->input_buffer.planes[0].stride;
    hw_layer.input_buffer.size = sdm_layer->input_buffer.size;
    hw_layer.input_buffer.acquire_fence_fd = sdm_layer->input_buffer.acquire_fence_fd;
  }

  return;
}

void DisplayBase::PostCommitLayerParams(LayerStack *layer_stack) {
  // Copy the release fence from HWLayers to clients layers
    uint32_t hw_layers_count = UINT32(hw_layers_.info.hw_layers.size());

  std::vector<uint32_t> fence_dup_flag;

  for (uint32_t i = 0; i < hw_layers_count; i++) {
    uint32_t sdm_layer_index = hw_layers_.info.index[i];
    Layer *sdm_layer = layer_stack->layers.at(sdm_layer_index);
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);

    // Copy the release fence only once for a SDM Layer.
    // In S3D use case, two hw layers can share the same input buffer, So make sure to merge the
    // output fence fd and assign it to layer's input buffer release fence fd.
    if (std::find(fence_dup_flag.begin(), fence_dup_flag.end(), sdm_layer_index) ==
        fence_dup_flag.end()) {
      sdm_layer->input_buffer.release_fence_fd = hw_layer.input_buffer.release_fence_fd;
      fence_dup_flag.push_back(sdm_layer_index);
    } else {
      int temp = -1;
      buffer_sync_handler_->SyncMerge(hw_layer.input_buffer.release_fence_fd,
                                      sdm_layer->input_buffer.release_fence_fd, &temp);

      if (hw_layer.input_buffer.release_fence_fd >= 0) {
        Sys::close_(hw_layer.input_buffer.release_fence_fd);
        hw_layer.input_buffer.release_fence_fd = -1;
      }

      if (sdm_layer->input_buffer.release_fence_fd >= 0) {
        Sys::close_(sdm_layer->input_buffer.release_fence_fd);
        sdm_layer->input_buffer.release_fence_fd = -1;
      }

      sdm_layer->input_buffer.release_fence_fd = temp;
    }

    // Reset the sync fence fds of HWLayer
    hw_layer.input_buffer.acquire_fence_fd = -1;
    hw_layer.input_buffer.release_fence_fd = -1;
  }

  return;
}

DisplayError DisplayBase::InitializeColorModes() {
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DisplayError error = color_mgr_->ColorMgrGetNumOfModes(&num_color_modes_);
  if (error != kErrorNone || !num_color_modes_) {
    DLOGV_IF(kTagQDCM, "GetNumModes failed = %d count = %d", error, num_color_modes_);
    return kErrorNotSupported;
  }
  DLOGI("Number of Color Modes = %d", num_color_modes_);

  if (!color_modes_.size()) {
    color_modes_.resize(num_color_modes_);

    DisplayError error = color_mgr_->ColorMgrGetModes(&num_color_modes_, color_modes_.data());
    if (error != kErrorNone) {
      color_modes_.clear();
      DLOGE("Failed");
      return error;
    }

    for (uint32_t i = 0; i < num_color_modes_; i++) {
      DLOGV_IF(kTagQDCM, "Color Mode[%d]: Name = %s mode_id = %d", i, color_modes_[i].name,
               color_modes_[i].id);
      auto it = color_mode_map_.find(color_modes_[i].name);
      if (it != color_mode_map_.end()) {
        if (it->second->id < color_modes_[i].id) {
          color_mode_map_.erase(it);
          color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
        }
      } else {
        color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
      }
    }
  }

  return kErrorNone;
}

DisplayError DisplayBase::HandleHDR(LayerStack *layer_stack) {
  DisplayError error = kErrorNone;

  if (display_type_ != kPrimary) {
    // Handling is needed for only primary displays
    return kErrorNone;
  }

  if (!layer_stack->flags.hdr_present) {
    //  HDR playback off - set prev mode
    if (hdr_playback_mode_) {
      hdr_playback_mode_ = false;
      if (color_mgr_ && !disable_hdr_lut_gen_) {
        // Do not apply HDR Mode when hdr lut generation is disabled
        DLOGI("Setting color mode = %s", current_color_mode_.c_str());
        //  HDR playback off - set prev mode
        error = SetColorModeInternal(current_color_mode_);
      }
      comp_manager_->ControlDpps(true);  // Enable Dpps
    }
  } else {
    // hdr is present
    if (!hdr_playback_mode_ && !layer_stack->flags.animating) {
      // hdr is starting
      hdr_playback_mode_ = true;
      if (color_mgr_ && !disable_hdr_lut_gen_) {
        DLOGI("Setting HDR color mode = %s", hdr_color_mode_.c_str());
        error = SetColorModeInternal(hdr_color_mode_);
      }
      comp_manager_->ControlDpps(false);  // Disable Dpps
    }
  }

  return error;
}

}  // namespace sdm
