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

'use strict'

import { Url } from './util/url'
import { RTSPConnection, RTSPWebSocketBackend } from './rtsp_new/connection';
import { RTSPClientSM } from './rtsp_new/client'

class Va {
  constructor ({url, wsUrl}, cb, clear) {
    this.url = url
    this.wsUrl = wsUrl
    this.cb = cb
    this.clear = clear
    this.isReplaced = url !== undefined
  }

  start () {
    let parsed = Url.parse(this.url)
    this.connection = new RTSPConnection(parsed.host, parsed.port, parsed.urlpath, {}, RTSPWebSocketBackend, this.wsUrl)
    this.client = new RTSPClientSM(this.connection)
    this.client.transitionTo(RTSPClientSM.STATE_OPTIONS);
    this.client.eventSource.addEventListener('sending', event => {
      this.cb(event.detail)
    })
    this.client.eventSource.addEventListener('clear', () => {
      this.clear()
    })
  }

  stop () {
    if (this.isReplaced) {
      if (this.client.currentState.name != RTSPClientSM.STATE_INITIAL) {
        this.client.transitionTo(RTSPClientSM.STATE_TEARDOWN);
      }
    }
  }
}

export default function initVa ({ url, wsUrl }, cb, clear) {
  let va = new Va({url, wsUrl}, cb, clear);
  va.start()
  return va.stop.bind(va)
}
