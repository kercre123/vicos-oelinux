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


#include <OMX_Core.h>
#include <OMX_Types.h>
#include <OMX_Component.h>
#include <utils/RefBase.h>

#include "qmmf_avcodec_common.h"

extern "C" {
typedef OMX_ERRORTYPE OMX_APIENTRY (*OMXGetHandle) (
    OMX_OUT OMX_HANDLETYPE* pHandle,
    OMX_IN  OMX_STRING cComponentName,
    OMX_IN  OMX_PTR pAppData,
    OMX_IN  OMX_CALLBACKTYPE* pCallBacks);
typedef OMX_ERRORTYPE OMX_APIENTRY (*OMXFreeHandle) (
    OMX_IN  OMX_HANDLETYPE hComponent);
typedef OMX_ERRORTYPE (*OMXGetComponentsOfRole) (
    OMX_IN      OMX_STRING role,
    OMX_INOUT   OMX_U32 *pNumComps,
    OMX_INOUT   OMX_U8  **compNames);
}

namespace qmmf {
namespace avcodec {

using namespace android;

typedef struct OMXContext_t {
  void *library_handle_;
  OMXGetHandle omx_get_handle_;
  OMXFreeHandle omx_free_handle_;
  OMXGetComponentsOfRole omx_get_components_of_role_;
} OMXContext;

class OmxClient : public RefBase {
public:
  OmxClient();

  ~OmxClient();

  OMX_ERRORTYPE CreateOmxHandle(char* component_name, void* app_data,
                                OMX_CALLBACKTYPE &callbacks);

  OMX_ERRORTYPE ReleaseOmxHandle();

  OMX_ERRORTYPE GetComponentsOfRole(OMX_STRING role, OMX_U32 *num_comps,
                                    OMX_U8 **comp_names);

  OMX_ERRORTYPE GetExtensionIndex(OMX_STRING param_name, OMX_INDEXTYPE* index);

  OMX_ERRORTYPE GetParameter(OMX_INDEXTYPE param_index, OMX_PTR param_data);

  OMX_ERRORTYPE SetParameter(OMX_INDEXTYPE param_index, OMX_PTR param_data);

  OMX_ERRORTYPE UseBuffer(OMX_BUFFERHEADERTYPE** buffer_hdr, OMX_U32 port,
                          OMX_PTR app_data, OMX_U32 size, OMX_U8* buffer);

  OMX_ERRORTYPE AllocateBuffer(OMX_BUFFERHEADERTYPE**  buffer_hdr, OMX_U32 port,
                               OMX_PTR  app_data, OMX_U32  bytes);

  OMX_ERRORTYPE FreeBuffer(OMX_BUFFERHEADERTYPE* buffer, OMX_U32 port);

  OMX_ERRORTYPE Flush(OMX_U32 nPortIndex);

  OMX_ERRORTYPE SendCommand(OMX_COMMANDTYPE cmd, OMX_U32 param, OMX_PTR cmd_data);

  OMX_ERRORTYPE GetConfig(OMX_INDEXTYPE config_index, OMX_PTR config_data);

  OMX_ERRORTYPE SetConfig(OMX_INDEXTYPE config_index, OMX_PTR config_data);

  OMX_ERRORTYPE EmptyThisBuffer(OMX_BUFFERHEADERTYPE *buffer);

  OMX_ERRORTYPE FillThisBuffer(OMX_BUFFERHEADERTYPE *buffer);

private:
  OMX_HANDLETYPE codec_handle_;
  OMXContext_t omx_context_;
  static const char kOMXPath[];
  static const char kOMXGetHandleName[];
  static const char kOMXFreeHandleName[];
  static const char kOMXGetComponentsOfRoleName[];
}; // class OmxClient

}; // namespace avcodec
}; // namespace qmmf
