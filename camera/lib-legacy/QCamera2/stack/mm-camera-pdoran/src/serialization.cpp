#include "serialization.h"

namespace anki {

std::ostream & operator << (std::ostream &out, const cam_metadata_info_t &value)
{
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_stats_valid:                         "<<(uint32_t)(value.is_stats_valid)<<std::endl;
  if (value.is_stats_valid)
    out<<"stats_data: "<<std::endl<<value.stats_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_faces_valid:                         "<<(uint32_t)(value.is_faces_valid)<<std::endl;
  if (value.is_faces_valid)
    out<<"faces_data: "<<std::endl<<value.faces_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_focus_valid:                         "<<(uint32_t)(value.is_focus_valid)<<std::endl;
  if (value.is_focus_valid)
    out<<"focus_data: "<<std::endl<<value.focus_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_crop_valid:                          "<<(uint32_t)(value.is_crop_valid)<<std::endl;
  if (value.is_crop_valid)
    out<<"crop_data: "<<std::endl<<value.focus_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_prep_snapshot_done_valid:            "<<(uint32_t)(value.is_prep_snapshot_done_valid)<<std::endl;
  if (value.is_prep_snapshot_done_valid)
    out<<"prep_snapshot_done_state: "<<std::endl<<value.prep_snapshot_done_state<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_good_frame_idx_range_valid:          "<<(uint32_t)(value.is_good_frame_idx_range_valid)<<std::endl;
  if (value.is_good_frame_idx_range_valid)
    out<<"good_frame_idx_range: "<<std::endl<<value.good_frame_idx_range<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_hdr_scene_data_valid:                "<<(uint32_t)(value.is_hdr_scene_data_valid)<<std::endl;
  if (value.is_hdr_scene_data_valid)
    out<<"hdr_scene_data: "<<std::endl<<value.hdr_scene_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_asd_decision_valid:                  "<<(uint32_t)(value.is_asd_decision_valid)<<std::endl;
  if (value.is_asd_decision_valid)
    out<<"scene: "<<std::endl<<value.scene<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_ae_params_valid:                     "<<(uint32_t)(value.is_ae_params_valid)<<std::endl;
  if (value.is_ae_params_valid)
    out<<"ae_params: "<<std::endl<<value.ae_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_awb_params_valid:                    "<<(uint32_t)(value.is_awb_params_valid)<<std::endl;
  if (value.is_awb_params_valid)
    out<<"awb_params: "<<std::endl<<value.awb_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_ae_exif_debug_valid:                 "<<(uint32_t)(value.is_ae_exif_debug_valid)<<std::endl;
  if (value.is_ae_exif_debug_valid)
    out<<"ae_exif_debug_params: "<<std::endl<<value.ae_exif_debug_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_awb_exif_debug_valid:                "<<(uint32_t)(value.is_awb_exif_debug_valid)<<std::endl;
  if (value.is_awb_exif_debug_valid)
    out<<"awb_exif_debug_params: "<<std::endl<<value.awb_exif_debug_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_af_exif_debug_valid:                 "<<(uint32_t)(value.is_af_exif_debug_valid)<<std::endl;
  if (value.is_af_exif_debug_valid)
    out<<"af_exif_debug_params: "<<std::endl<<value.af_exif_debug_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_asd_exif_debug_valid:                "<<(uint32_t)(value.is_asd_exif_debug_valid)<<std::endl;
  if (value.is_asd_exif_debug_valid)
    out<<"asd_exif_debug_params: "<<std::endl<<value.asd_exif_debug_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_stats_buffer_exif_debug_valid:       "<<(uint32_t)(value.is_stats_buffer_exif_debug_valid)<<std::endl;
  if (value.is_stats_buffer_exif_debug_valid)
    out<<"stats_buffer_exif_debug_params: "<<std::endl<<value.stats_buffer_exif_debug_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_sensor_params_valid:                 "<<(uint32_t)(value.is_sensor_params_valid)<<std::endl;
  if (value.is_sensor_params_valid)
    out<<"sensor_params: "<<std::endl<<value.sensor_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_meta_invalid:                        "<<(uint32_t)(value.is_meta_invalid)<<std::endl;
  if (value.is_meta_invalid)
    out<<"meta_invalid_params: "<<std::endl<<value.meta_invalid_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_preview_frame_skip_valid:            "<<(uint32_t)(value.is_preview_frame_skip_valid)<<std::endl;
  if (value.is_preview_frame_skip_valid)
    out<<"preview_frame_skip_idx_range: "<<std::endl<<value.preview_frame_skip_idx_range<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_tuning_params_valid:                 "<<(uint32_t)(value.is_tuning_params_valid)<<std::endl;
  if (value.is_tuning_params_valid)
    out<<"tuning_params: "<<std::endl<<value.tuning_params<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_chromatix_mobicat_af_valid:          "<<(uint32_t)(value.is_chromatix_mobicat_af_valid)<<std::endl;
  if (value.is_chromatix_mobicat_af_valid)
    out<<"chromatix_mobicat_af_data: "<<std::endl<<value.chromatix_mobicat_af_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_chromatix_lite_isp_valid:            "<<(uint32_t)(value.is_chromatix_lite_isp_valid)<<std::endl;
  if (value.is_chromatix_lite_isp_valid)
    out<<"chromatix_lite_isp_data: "<<std::endl<<value.chromatix_lite_isp_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_chromatix_lite_pp_valid:             "<<(uint32_t)(value.is_chromatix_lite_pp_valid)<<std::endl;
  if (value.is_chromatix_lite_pp_valid)
    out<<"chromatix_lite_pp_data: "<<std::endl<<value.chromatix_lite_pp_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_chromatix_lite_ae_stats_valid:       "<<(uint32_t)(value.is_chromatix_lite_ae_stats_valid)<<std::endl;
  if (value.is_chromatix_lite_ae_stats_valid)
    out<<"chromatix_lite_ae_stats_data: "<<std::endl<<value.chromatix_lite_ae_stats_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_chromatix_lite_awb_stats_valid:      "<<(uint32_t)(value.is_chromatix_lite_awb_stats_valid)<<std::endl;
  if (value.is_chromatix_lite_awb_stats_valid)
    out<<"chromatix_lite_awb_stats_data: "<<std::endl<<value.chromatix_lite_awb_stats_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_chromatix_lite_af_stats_valid:       "<<(uint32_t)(value.is_chromatix_lite_af_stats_valid)<<std::endl;
  if (value.is_chromatix_lite_af_stats_valid)
    out<<"chromatix_lite_af_stats_data: "<<std::endl<<value.chromatix_lite_af_stats_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_mobicat_ae_params_valid:             "<<(uint32_t)(value.is_mobicat_ae_params_valid)<<std::endl;
  if (value.is_mobicat_ae_params_valid)
    out<<"mobicat_ae_data: "<<std::endl<<value.mobicat_ae_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_mobicat_awb_params_valid:            "<<(uint32_t)(value.is_mobicat_awb_params_valid)<<std::endl;
  if (value.is_mobicat_awb_params_valid)
    out<<"mobicat_awb_data: "<<std::endl<<value.mobicat_awb_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_mobicat_af_params_valid:             "<<(uint32_t)(value.is_mobicat_af_params_valid)<<std::endl;
  if (value.is_mobicat_af_params_valid)
    out<<"mobicat_af_data: "<<std::endl<<value.mobicat_af_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_mobicat_asd_params_valid:            "<<(uint32_t)(value.is_mobicat_asd_params_valid)<<std::endl;
  if (value.is_mobicat_asd_params_valid)
    out<<"mobicat_asd_data: "<<std::endl<<value.mobicat_asd_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_mobicat_stats_params_valid:          "<<(uint32_t)(value.is_mobicat_stats_params_valid)<<std::endl;
  if (value.is_mobicat_stats_params_valid)
    out<<"mobicat_stats_buffer_data: "<<std::endl<<value.mobicat_stats_buffer_data<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_focus_pos_info_valid:                "<<(uint32_t)(value.is_focus_pos_info_valid)<<std::endl;
  if (value.is_focus_pos_info_valid)
    out<<"cur_pos_info: "<<std::endl<<value.cur_pos_info<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;
  out<<"is_frame_id_reset:                      "<<(uint32_t)(value.is_frame_id_reset)<<std::endl;
  out<<"--------------------------------------------------"<<std::endl;

  return out;
}

std::ostream & operator << (std::ostream &out, const cam_hist_stats_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_face_detection_data_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_auto_focus_data_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_crop_data_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_prep_snapshot_state_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_frame_idx_range_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_asd_hdr_scene_data_t &value)
{
  out << "is_hdr_scene:        "<<value.is_hdr_scene<<std::endl;
  out << "hdr_confidence:      "<<value.hdr_confidence<<std::endl;
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_auto_scene_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_ae_params_t &value)
{
  out << "exp_time:             "<<value.exp_time<<std::endl;
  out << "real_gain:            "<<value.real_gain<<std::endl;
  out << "iso_value:            "<<value.iso_value<<std::endl;
  out << "flash_needed:         "<<value.flash_needed<<std::endl;
  out << "settled:              "<<value.settled<<std::endl;
  out << "exp_index:            "<<value.exp_index<<std::endl;
  out << "line_count:           "<<value.line_count<<std::endl;
  out << "metering_mode:        "<<value.metering_mode<<std::endl;
  out << "exposure_program:     "<<value.exposure_program<<std::endl;
  out << "exposure_mode:        "<<value.exposure_mode<<std::endl;
  out << "scenetype:            "<<value.scenetype<<std::endl;
  out << "brightness:           "<<value.brightness<<std::endl;
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_awb_params_t &value)
{
  out << "cct_value:        "<<value.cct_value<<std::endl;
  // out << "rgb_gains:        "<<value.rgb_gains<<std::endl;
  out << "decision:         "<<value.decision<<std::endl;
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_ae_exif_debug_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_awb_exif_debug_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_af_exif_debug_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_asd_exif_debug_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_stats_buffer_exif_debug_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_sensor_params_t &value)
{
  out << "flash_mode:             "<<(int)value.flash_mode<<std::endl;
  out << "sens_type:              "<<(int)value.sens_type<<std::endl;
  out << "aperture_value:         "<<value.aperture_value<<std::endl;
  out << "focal_length:           "<<value.focal_length<<std::endl;
  out << "f_number:               "<<value.f_number<<std::endl;
  out << "sensing_method:         "<<value.sensing_method<<std::endl;
  out << "crop_factor:            "<<value.crop_factor<<std::endl;
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_meta_valid_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const tuning_params_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_chromatix_mobicat_af_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_isp_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_pp_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_ae_stats_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_awb_stats_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_af_stats_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_focus_pos_info_t &value)
{
  return out;
}

std::ostream & operator << (std::ostream &out, const cam_awb_gain_t &value)
{
  out<<"["<<value.r_gain<<", "<<value.g_gain<<", "<<value.b_gain<<"]";
  return out;
}

 

} /* namespace anki */