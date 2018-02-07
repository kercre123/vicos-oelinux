/*
* Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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
#ifndef QMMF_VAM_CONFIG_PARSER_H_
#define QMMF_VAM_CONFIG_PARSER_H_

#include <utils/String8.h>
#include <json_configuration.h>
#include <VAM/vaapi_config.h>

namespace qmmf {

namespace vaminterface {

using namespace android;

class VAMConfigParser {
 public:
  VAMConfigParser();
  virtual ~VAMConfigParser();
  int32_t Init();
  int32_t ParseConfig(const char *json_config,
                      struct vaapi_configuration &result);

 private:

  int32_t ConvertZones(JSONVAZone *json_zones, vaapi_zone *vam_zones,
                       size_t count);

  /**Not allowed */
  VAMConfigParser(const VAMConfigParser &);
  VAMConfigParser &operator=(const VAMConfigParser &);

  void *parser_handle_;
};

} //namespace vaminterface ends
} //namespace qmmf ends
#endif /* QMMF_VAM_CONFIG_PARSER_H_ */
