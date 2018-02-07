/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/Log.h>
extern "C"{
#include <configuration_parser_apis.h>
}
#include "qmmf_vam_config_parser.h"

namespace qmmf {

namespace vaminterface {

VAMConfigParser::VAMConfigParser() : parser_handle_(NULL) {}

VAMConfigParser::~VAMConfigParser() {
  if (NULL != parser_handle_) {
    configuration_parser_deinit(parser_handle_);
    parser_handle_ = NULL;
  }
}

int32_t VAMConfigParser::Init() {
  int32_t ret = NO_ERROR;

  auto status = configuration_parser_init(&parser_handle_);
  if (JSON_VA_OK != status) {
    ALOGE("%s: Failed to initialize parser!\n", __func__);
    ret = UNKNOWN_ERROR;
  }

  return ret;
}

int32_t VAMConfigParser::ConvertZones(JSONVAZone *json_zones,
                                      vaapi_zone *vam_zones, size_t count) {
    if((json_zones == NULL) || (vam_zones == NULL) ||
        (count == 0)) {
        return BAD_VALUE;
    }

    for(size_t i = 0; i < count; i++) {
        memset(vam_zones[i].id, '\0', sizeof(vam_zones[i].id));
        strncpy(vam_zones[i].id, json_zones[i].id, sizeof(vam_zones[i].id) - 1);
        size_t point_count = MIN(json_zones[i].cnt_points, VAAPI_POINTS_MAX);
        vam_zones[i].point_size = point_count;
        for(size_t j = 0; j < point_count; j++) {
            vam_zones[i].points[j].x = json_zones[i].points[j].x;
            vam_zones[i].points[j].y = json_zones[i].points[j].y;
        }

        switch(json_zones[i].type) {
          case ZONE_TYPE_INCLUDE:
            vam_zones[i].zone_type = vaapi_include_zone;
            break;
          case ZONE_TYPE_EXCLUDE:
            vam_zones[i].zone_type = vaapi_exclude_zone;
            break;
          case LINE_TYPE_BOTH_DIR:
            vam_zones[i].zone_type = vaapi_line;
            break;
          case LINE_TYPE_DIR:
            vam_zones[i].zone_type = vaapi_line_dir;
            break;
          default:
            ALOGE("%s: Unsupported zone type %d\n",
                  __func__, json_zones[i].type);
            break;
        };
    }

    return NO_ERROR;
}

int32_t VAMConfigParser::ParseConfig(const char *json_config,
                                     struct vaapi_configuration &result) {
  int32_t ret;
  struct JSONConfiguration *config = NULL;
  vaapi_zone zones[VAAPI_ZONE_MAX];

  if (NULL == parser_handle_) {
    ALOGE("%s: Parser not initialized!\n", __func__);
    return NO_INIT;
  }

  if (NULL == json_config) {
    return BAD_VALUE;
  }

  auto status = configuration_parser_clear(parser_handle_);
  if (JSON_VA_OK != status) {
    ALOGE("%s: Failed to clear parser!\n", __func__);
    return UNKNOWN_ERROR;
  }

  status = configuration_parser_parse_doc(parser_handle_, json_config, &config);
  if (JSON_VA_OK != status) {
    ALOGE("%s: Failed to parse configuration!\n", __func__);
    return UNKNOWN_ERROR;
  }

  if (NULL == config) {
    ALOGE("%s: Parsing failed, configuration invalid!\n", __func__);
    return UNKNOWN_ERROR;
  }

  memset(&result, 0, sizeof(result));
  size_t zone_count = MIN(config->cnt_zones, VAAPI_ZONE_MAX);
  if (0 < zone_count) {
    ret = ConvertZones(config->zones, zones, zone_count);
    if (NO_ERROR != ret) {
      ALOGE("%s: Unable to convert zones!\n", __func__);
      return ret;
    }
  }

  memset(result.version, '\0', sizeof(result.version));
  strncpy(result.version, config->version, sizeof(result.version) - 1);
  memset(result.id, '\0', sizeof(result.id));
  strncpy(result.id, config->id, sizeof(result.id) - 1);

  result.rule_size = MIN(config->cnt_atomic_rules, VAAPI_RULE_MAX);
  if (0 < result.rule_size) {
    size_t id_size = sizeof(result.rules[0].id);
    size_t name_size = sizeof(result.rules[0].name);
    for (size_t i = 0; i < result.rule_size; i++) {
      result.rules[i].minimum_size = config->atomic_rules[i].min_size;
      result.rules[i].sensitivity = config->atomic_rules[i].sensitivity;
      memset(result.rules[i].id, '\0', id_size);
      strncpy(result.rules[i].id, config->atomic_rules[i].id, id_size - 1);
      memset(result.rules[i].name, '\0', name_size);
      strncpy(result.rules[i].name, config->atomic_rules[i].name,
              name_size- 1);

      result.rules[i].type = static_cast<vaapi_event_type> (
          config->atomic_rules[i].event_type);
      result.rules[i].zone_size = MIN(config->atomic_rules[i].cnt_zones,
                                      VAAPI_ZONE_MAX);
      memcpy(result.rules[i].reserve, config->atomic_rules[i].reserve,
              sizeof(result.rules[i].reserve));
      memcpy(result.rules[i].reserve_str, config->atomic_rules[i].reserve_str,
              sizeof(result.rules[i].reserve_str));
      if (0 < result.rules[i].zone_size) {
        for (size_t j = 0; j < result.rules[i].zone_size; j++) {
          for (size_t k = 0; k < config->cnt_zones; k++) {
            if (0 == strcmp(zones[k].id, config->atomic_rules[i].zone_ids[j])) {
              result.rules[i].zones[j] = zones[k];
              break;
            }
          }
        }
      }
    }
  }

  return ret;
}

} //namespace vaminterface ends
} //namespace qmmf ends
