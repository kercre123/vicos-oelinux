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

import { debug } from 'src/util'

export function initVlc (TAG, id) {
  let vlc = document.getElementById(id)
  debug(TAG)
  vlc.addEventListener('MediaPlayerBuffering', e => {
    if (e) {
      debug(TAG, 'Buffer: ' + e + '%')
    } else {
      debug(TAG, 'Nothing be buffering...')
    }
  }, false)
  vlc.addEventListener('MediaPlayerEncounteredError', (e) => {
    debug('MediaPlayerEncounteredError', 'MediaPlayerEncounteredError')
    vlc.playlist.stop()
    vlc.playlist.items.clear()
    let msg = 'VLC Web Plugin encountered error, please reload page.'
    layer.confirm(msg, {
      icon: 0,
      title: ['alarm', true],
      area: '450px',
      btn: ['OK', 'Cancel']
    }, function () {
      window.location.reload()
    })
  }, false)

  return vlc
}

export function initChromePlayer (TAG ,id) {
  return document.getElementById(id)
}
