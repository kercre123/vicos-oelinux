/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
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

#define TAG "RecorderPostProcFactory"

#include <dirent.h>

#include "../modules/camera-hal-jpeg/qmmf_camera_hal_jpeg.h"
#include "../modules/camera-hal-reproc/qmmf_camera_hal_reproc.h"
#include "../modules/jpeg-encoder/qmmf_jpeg.h"
#include "../modules/test/qmmf_postproc_test.h"
#include "../modules/algo/qmmf_postproc_algo.h"

#include "qmmf_postproc_factory.h"

namespace qmmf {

namespace recorder {

sp<PostProcFactory> PostProcFactory::instance_ = nullptr;
int32_t PostProcFactory::ids_ = 0x00f00000;

const std::string PostProcFactory::plugin_prefix = "libqmmf_alg";
const std::string PostProcFactory::plugin_suffix = ".so";

sp<PostProcFactory> PostProcFactory::getInstance() {
  if (instance_.get() == nullptr) {
    instance_ = new PostProcFactory();
  }
  return instance_;
}

void PostProcFactory::releaseInstance() {
  QMMF_INFO("%s: destroying instance object.", __func__);
  if (instance_.get() == nullptr) {
    QMMF_INFO("%s: Reset reprocess Ids.", __func__);
    ids_ = 0x00f00000;
  }
  instance_.clear();
}

PostProcFactory::PostProcFactory() {
}

PostProcFactory::~PostProcFactory() {
}

int32_t PostProcFactory::GetUniqueId() {
  return ids_++;
}

status_t PostProcFactory::GetSupportedPlugins(SupportedPlugins *plugins) {

  std::vector<std::string> libraries;
  auto plugins_path = Utils::GetAlgLibFolder();

  DIR *dp = opendir(plugins_path.c_str());
  if (nullptr == dp) {
    QMMF_ERROR("%s:%s: Failed to open plugins folder: %s", TAG, __func__,
        strerror(errno));
    return PERMISSION_DENIED;
  }
  struct dirent *entry;
  while ((entry = readdir(dp)) != nullptr) {
    if (IsPlugin(entry->d_name)) {
      std::string library(plugins_path);
      library.append(entry->d_name);
      libraries.push_back(library);
    }
  }
  closedir(dp);

  for (auto const& library : libraries) {
    try {
      void *lib_handle;
      Utils::LoadLib(library, lib_handle);

      QmmfAlgLoadPlugin LoadPluginFunc;
      Utils::LoadLibHandler(lib_handle, QMMF_ALG_LIB_LOAD_FUNC, LoadPluginFunc);

      std::vector<uint8_t> calibration_data;
      auto plugin = LoadPluginFunc(calibration_data);

      auto caps = plugin->GetCaps();
      PluginInfo plugin_info(caps.plugin_name_, caps.lib_version_,
                      caps.runtime_enable_disable_);
      plugins->push_back(plugin_info);
      plugin_libraries_.emplace(plugin_info.name, library);

      Utils::UnloadLib(lib_handle);
    } catch (const std::exception &e) {
      QMMF_ERROR("%s:%s: Error getting plugin info for %s exception: %s", TAG,
          __func__, library.c_str(), e.what());
      return FAILED_TRANSACTION;
    }
  }

  return NO_ERROR;
}

status_t
PostProcFactory::CreatePlugin(uint32_t &uid, const PluginInfo &plugin) {
  if (plugin_libraries_.find(plugin.name) != plugin_libraries_.end()) {
    uid = GetUniqueId();
    std::string library = plugin_libraries_.at(plugin.name);

    sp<IPostProcModule> module = new PostProcAlg(library);
    if (module.get() != nullptr) {
      sp<PostProcNode> node = new PostProcNode(uid, plugin.name, module);
      plugin_nodes_.emplace(uid, node);
    } else {
      QMMF_ERROR("%s: Failed to create plugin: %s", __func__,
          plugin.name.c_str());
      return FAILED_TRANSACTION;
    }
  } else {
    QMMF_ERROR("%s: Invalid plugin name: %s", __func__, plugin.name.c_str());
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t PostProcFactory::DeletePlugin(const uint32_t &uid) {
  if (plugin_nodes_.find(uid) != plugin_nodes_.end()) {
    if (plugin_nodes_in_use_.find(uid) != plugin_nodes_in_use_.end()) {
      QMMF_ERROR("%s: Plugin(%d) is still used by a pipeline!", __func__, uid);
      return INVALID_OPERATION;
    }
    plugin_nodes_.at(uid).clear();
    plugin_nodes_.erase(uid);
  } else {
    QMMF_ERROR("%s: Invalid plugin uid: %d", __func__, uid);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t PostProcFactory::ConfigPlugin(const uint32_t &uid,
                                       const std::string &config) {
  if (plugin_nodes_.find(uid) != plugin_nodes_.end()) {
    auto ret = plugin_nodes_.at(uid)->Configure(config);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Failed to configure plugin uid: %d", __func__, uid);
      return ret;
    }
  } else {
    QMMF_ERROR("%s: Invalid plugin uid: %d", __func__, uid);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

sp<PostProcNode>
PostProcFactory::GetProcNode(const uint32_t &uid) {
  sp<PostProcNode> node;

  if (plugin_nodes_.find(uid) != plugin_nodes_.end()) {
    if (plugin_nodes_in_use_.find(uid) == plugin_nodes_in_use_.end()) {
      node = plugin_nodes_.at(uid);
      plugin_nodes_in_use_.insert(uid);
    } else {
      QMMF_ERROR("%s: Plugin node is already in use: %d", __func__, uid);
    }
  } else {
    QMMF_ERROR("%s: Invalid post process engine: %d", __func__, uid);
  }
  return node;
}

sp<PostProcNode>
PostProcFactory::GetProcNode(const std::string &name, IPostProc* context) {
  sp<PostProcNode> node;
  sp<IPostProcModule> module;

  if (name == "JpegEncode") {
    module = new PostProcJpeg();
  } else if (name == "HALJpegEncode") {
    module = new PostProcHalJpeg(context);
  } else if (name == "HALReprocess") {
    module = new CameraHalReproc(context);
  } else if (name == "Test") {
    module = new PostProcTest();
  } else {
    QMMF_ERROR("%s: Invalid post process engine: %s", __func__, name.c_str());
    return node;
  }

  if (module.get() != nullptr) {
    node = new PostProcNode(GetUniqueId(), name, module);
    internal_nodes_.emplace(node->GetId(), node);
  } else {
    QMMF_ERROR("%s: Failed to create module: %s", __func__, name.c_str());
    return node;
  }
  return node;
}

status_t PostProcFactory::ReturnProcNode(const uint32_t &uid) {
  if (plugin_nodes_in_use_.find(uid) != plugin_nodes_in_use_.end()) {
    plugin_nodes_in_use_.erase(uid);
  } else if (internal_nodes_.find(uid) != internal_nodes_.end()) {
    internal_nodes_.erase(uid);
  } else {
    QMMF_ERROR("%s: Invalid node id: %d", __func__, uid);
  }

  return NO_ERROR;
}

bool PostProcFactory::IsPlugin(std::string entry) {
  /* Ignore "." and ".." */
  if (entry.compare(".") == 0 || entry.compare("..") == 0) {
    return false;
  }
  /* Ignore entries that are too short */
  if (entry.size() <= (plugin_prefix.size() + plugin_suffix.size())) {
    return false;
  }
  /* Check prefix */
  if (entry.compare(0, plugin_prefix.size(), plugin_prefix) != 0) {
    return false;
  }
  /* Check suffix */
  size_t pos = entry.size() - plugin_suffix.size();
  if (entry.compare(pos, plugin_suffix.size(), plugin_suffix) != 0) {
    return false;
  }
  return true;
}

}; // namespace recoder

}; // namespace qmmf
