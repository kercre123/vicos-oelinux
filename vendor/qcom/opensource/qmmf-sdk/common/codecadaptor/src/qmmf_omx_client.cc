/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *     Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.

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

#define TAG "QMMF_OMX_CLIENT"

#include <dlfcn.h>
#include "common/qmmf_log.h"
#include "qmmf_omx_client.h"

namespace qmmf {
namespace avcodec {

const char OmxClient::kOMXPath[] = "libOmxCore.so";
const char OmxClient::kOMXGetHandleName[] = "OMX_GetHandle";
const char OmxClient::kOMXFreeHandleName[] = "OMX_FreeHandle";
const char OmxClient::kOMXGetComponentsOfRoleName[] =
    "OMX_GetComponentsOfRole";

OmxClient::OmxClient():codec_handle_(NULL), omx_context_() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

OmxClient::~OmxClient() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  ReleaseOmxHandle();
  QMMF_INFO("%s:%s: Exit", TAG, __func__);
}

OMX_ERRORTYPE OmxClient::CreateOmxHandle(char* Codecname, void* app_data,
                                         OMX_CALLBACKTYPE &callbacks) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  if (nullptr == omx_context_.library_handle_) {
    omx_context_.library_handle_ = dlopen(kOMXPath, RTLD_NOW);
    if (omx_context_.library_handle_ == nullptr) {
      char const *err_str = dlerror();
      QMMF_ERROR("%s:%s load: module=%s\n%s \n", TAG, __func__, kOMXPath,
                 err_str ? err_str : "unknown");
      ret = OMX_ErrorComponentNotFound;
      goto exit;
    }

    omx_context_.omx_get_handle_ = (OMXGetHandle)dlsym(
        omx_context_.library_handle_, kOMXGetHandleName);
    if (nullptr == omx_context_.omx_get_handle_) {
      QMMF_ERROR("%s:%s load: couldn't find symbol %s\n", TAG, __func__,
          kOMXGetHandleName);
      ret = OMX_ErrorComponentNotFound;
      goto exit;
    }

    omx_context_.omx_free_handle_ = (OMXFreeHandle)dlsym(
        omx_context_.library_handle_, kOMXFreeHandleName);
    if (nullptr == omx_context_.omx_free_handle_) {
      QMMF_ERROR("%s:%s load: couldn't find symbol %s\n", TAG, __func__,
          kOMXFreeHandleName);
      ret = OMX_ErrorComponentNotFound;
      goto exit;
    }

    omx_context_.omx_get_components_of_role_ = (OMXGetComponentsOfRole)dlsym(
        omx_context_.library_handle_, kOMXGetComponentsOfRoleName);
    if (NULL == omx_context_.omx_free_handle_) {
      QMMF_ERROR("%s:%s load: couldn't find symbol %s\n", TAG, __func__,
          kOMXGetComponentsOfRoleName);
      ret = OMX_ErrorComponentNotFound;
      goto exit;
    }
  }
  ret = omx_context_.omx_get_handle_(&codec_handle_,
                                     const_cast<char *>(Codecname),
                                     app_data, &callbacks);
  QMMF_INFO("%s:%s created OMX handle(%p)", TAG, __func__, codec_handle_);

  return ret;

exit:

  if (nullptr != omx_context_.library_handle_) {
    dlclose(omx_context_.library_handle_);
    omx_context_.library_handle_ = nullptr;
  }

   return ret;
}

OMX_ERRORTYPE OmxClient::ReleaseOmxHandle() {

  QMMF_INFO("%s:%s: Enter ", TAG, __func__);
  OMX_ERRORTYPE ret = OMX_ErrorNone;
  if(codec_handle_ && (nullptr != omx_context_.omx_free_handle_)) {
    QMMF_INFO("%s: Free OMX handle(%p)", __func__, codec_handle_);
    ret = omx_context_.omx_free_handle_(codec_handle_);
    codec_handle_ = nullptr;
  }

  if(nullptr != omx_context_.library_handle_) {
    dlclose(omx_context_.library_handle_);
    memset(&omx_context_, 0, sizeof(omx_context_));
  }

  QMMF_INFO("%s:%s: Exit", TAG, __func__);

  return ret;
}

OMX_ERRORTYPE OmxClient::GetComponentsOfRole(OMX_STRING role,
                                             OMX_U32 *num_comps,
                                             OMX_U8 **comp_names) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  if (nullptr != omx_context_.omx_get_components_of_role_) {
    ret = omx_context_.omx_get_components_of_role_(role, num_comps,
                                                   comp_names);
  } else {
    ret = OMX_ErrorComponentNotFound;
  }

  return ret;
}

OMX_ERRORTYPE OmxClient::GetExtensionIndex(OMX_STRING param_name, OMX_INDEXTYPE* index) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_GetExtensionIndex(codec_handle_, param_name, index);

  return ret;
}

OMX_ERRORTYPE OmxClient::GetParameter(OMX_INDEXTYPE param_index,
                                      OMX_PTR param_data) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret =  OMX_GetParameter(codec_handle_, param_index, param_data);

  return ret;
}

OMX_ERRORTYPE OmxClient::SetParameter(OMX_INDEXTYPE param_index,
                                      OMX_PTR param_data) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_SetParameter(codec_handle_, param_index, param_data);

  return ret;
}


OMX_ERRORTYPE OmxClient::AllocateBuffer(OMX_BUFFERHEADERTYPE** buffer_hdr,
                                        OMX_U32 port, OMX_PTR app_data,
                                        OMX_U32 size) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_AllocateBuffer(codec_handle_, buffer_hdr, port, app_data, size);

  return ret;
}

OMX_ERRORTYPE OmxClient::UseBuffer(OMX_BUFFERHEADERTYPE** buffer_hdr,
                                   OMX_U32 port, OMX_PTR app_data,
                                   OMX_U32 size, OMX_U8 *buffer) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_UseBuffer(codec_handle_, buffer_hdr, port, app_data, size, buffer);

  return ret;
}

OMX_ERRORTYPE OmxClient::FreeBuffer(OMX_BUFFERHEADERTYPE* buffer_hdr,
                                    OMX_U32 port) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_FreeBuffer(codec_handle_, port, buffer_hdr);

  return ret;
}

OMX_ERRORTYPE OmxClient::SendCommand(OMX_COMMANDTYPE cmd, OMX_U32 port_index,
                                     OMX_PTR cmd_data) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  switch(cmd) {
    case OMX_CommandStateSet:
      ret = OMX_SendCommand(codec_handle_, OMX_CommandStateSet, port_index,
                            cmd_data);
      break;
    case OMX_CommandFlush:
      ret = OMX_SendCommand(codec_handle_, OMX_CommandFlush, port_index,
                            cmd_data);
      break;
    case OMX_CommandPortDisable:
      ret = OMX_SendCommand(codec_handle_, OMX_CommandPortDisable, port_index,
                            cmd_data);
      break;
    case OMX_CommandPortEnable:
      ret = OMX_SendCommand(codec_handle_, OMX_CommandPortEnable, port_index,
                            cmd_data);
      break;
    default:
      QMMF_ERROR("%s:%s Unknown omx command", TAG, __func__);
      ret = OMX_ErrorBadParameter;
  }

  return ret;
}

OMX_ERRORTYPE OmxClient::GetConfig(OMX_INDEXTYPE config_index,
                                   OMX_PTR config_data) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_GetConfig(codec_handle_, config_index, config_data);

  return ret;
}

OMX_ERRORTYPE OmxClient::SetConfig(OMX_INDEXTYPE config_index,
                                   OMX_PTR config_data) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_SetConfig(codec_handle_, config_index, config_data);

  return ret;
}

OMX_ERRORTYPE OmxClient::EmptyThisBuffer(OMX_BUFFERHEADERTYPE *buffer) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_EmptyThisBuffer(codec_handle_, buffer);

  return ret;
}

OMX_ERRORTYPE OmxClient::FillThisBuffer(OMX_BUFFERHEADERTYPE *buffer) {

  OMX_ERRORTYPE ret = OMX_ErrorNone;
  ret = OMX_FillThisBuffer(codec_handle_, buffer);

  return ret;
}

}; // namespace avcodec
}; // namespace qmmf
