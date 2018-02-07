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

#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>

#include "../interface/qmmf_postproc_module.h"
#include "../interface/qmmf_postproc.h"
#include "../node/qmmf_postproc_node.h"

namespace qmmf {

namespace recorder {

class PostProcFactory : public RefBase {

 public:

   static sp<PostProcFactory> getInstance();

   static void releaseInstance();
   status_t GetSupportedPlugins(SupportedPlugins *plugins);

   status_t CreatePlugin(uint32_t &uid, const PluginInfo &info);

   status_t DeletePlugin(const uint32_t &uid);

   status_t ConfigPlugin(const uint32_t &uid, const std::string &config);

   sp<PostProcNode> GetProcNode(const uint32_t &uid);

   sp<PostProcNode> GetProcNode(const std::string &name, IPostProc* context);

   status_t ReturnProcNode(const uint32_t &uid);

 private:

   PostProcFactory();

   ~PostProcFactory();

   int32_t GetUniqueId();

   bool IsPlugin(std::string entry);

   static sp<PostProcFactory>    instance_;
   static int32_t                ids_;

   static const std::string      plugin_prefix;
   static const std::string      plugin_suffix;

   SupportedPlugins supported_plugins_;
   std::map<std::string, std::string> plugin_libraries_;

   std::map<uint32_t, sp<PostProcNode> > plugin_nodes_;
   std::set<uint32_t> plugin_nodes_in_use_;

   std::map<uint32_t, sp<PostProcNode> > internal_nodes_;
};

}; //namespace recorder

}; //namespace qmmf
