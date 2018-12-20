#ifndef _mm_pdoran_serialization_h_
#define _mm_pdoran_serialization_h_

#include <ostream>
#include <istream>

extern "C" { 
#include "cam_types.h"
}

namespace anki
{

std::ostream & operator << (std::ostream &out, const cam_metadata_info_t &value);
std::ostream & operator << (std::ostream &out, const cam_hist_stats_t &value);
std::ostream & operator << (std::ostream &out, const cam_face_detection_data_t &value);
std::ostream & operator << (std::ostream &out, const cam_auto_focus_data_t &value);
std::ostream & operator << (std::ostream &out, const cam_crop_data_t &value);
std::ostream & operator << (std::ostream &out, const cam_prep_snapshot_state_t &value);
std::ostream & operator << (std::ostream &out, const cam_frame_idx_range_t &value);
std::ostream & operator << (std::ostream &out, const cam_asd_hdr_scene_data_t &value);
std::ostream & operator << (std::ostream &out, const cam_auto_scene_t &value);
std::ostream & operator << (std::ostream &out, const cam_ae_params_t &value);
std::ostream & operator << (std::ostream &out, const cam_awb_params_t &value);
std::ostream & operator << (std::ostream &out, const cam_ae_exif_debug_t &value);
std::ostream & operator << (std::ostream &out, const cam_awb_exif_debug_t &value);
std::ostream & operator << (std::ostream &out, const cam_af_exif_debug_t &value);
std::ostream & operator << (std::ostream &out, const cam_asd_exif_debug_t &value);
std::ostream & operator << (std::ostream &out, const cam_stats_buffer_exif_debug_t &value);
std::ostream & operator << (std::ostream &out, const cam_sensor_params_t &value);
std::ostream & operator << (std::ostream &out, const cam_meta_valid_t &value);
std::ostream & operator << (std::ostream &out, const tuning_params_t &value);
std::ostream & operator << (std::ostream &out, const cam_chromatix_mobicat_af_t &value);
std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_isp_t &value);
std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_pp_t &value);
std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_ae_stats_t &value);
std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_awb_stats_t &value);
std::ostream & operator << (std::ostream &out, const cam_chromatix_lite_af_stats_t &value);
std::ostream & operator << (std::ostream &out, const cam_focus_pos_info_t &value);

} /* namespace anki */

#endif /* _mm_pdoran_serialization_h_ */