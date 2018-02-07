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

import { ajax } from 'src/util'
import { uri } from 'src/settings'

const GET = 'get'
const POST = 'post'

export function sendLogin (username, password) {
  let args = {
    methodName: 'Login',
    url: uri.login,
    method: POST,
    data: {
      username,
      userpwd: password
    }
  }
  return ajax(args)
}

export function sendLogout () {
  let args = {
    methodName: 'Logout',
    url: uri.logout,
    method: POST,
    data: {}
  }
  return ajax(args)
}

export function sendGetChannel () {
  let args = {
    methodName: 'Get Channel',
    url: uri.channel,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostChannel (channelSelectVal) {
  let args = {
    methodName: 'Post Channel',
    url: uri.channel,
    method: POST,
    data: {
      channelSelectVal
    }
  }
  return ajax(args)
}

export function sendGetVideoSetting () {
  let args = {
    methodName: 'Get Video Setting',
    url: uri.video,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostVideoSetting (setting) {
  const {
    resolutionSelected,
    encodeModeSelected,
    bitRateSelected,
    fpsSelected
  } = setting
  let args = {
    methodName: 'Post Video Setting',
    url: uri.video,
    method: POST,
    data: {
      resolutionSelectVal: resolutionSelected,
      encodeModeSelectVal: encodeModeSelected,
      bitRateSelectVal: bitRateSelected,
      fpsSelectVal: fpsSelected
    }
  }

  return ajax(args)
}

export function sendGetParams () {
  let args = {
    methodName: 'Get Video Params',
    url: uri.params,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostParams (data) {
  let args = {
    methodName: 'Post Video Params',
    url: uri.params,
    method: POST,
    data
  }
  return ajax(args)
}

export function sendGetPreview () {
  let args = {
    methodName: 'Get Preview',
    url: uri.preview,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostPreview (switchStatus) {
  let args = {
    methodName: 'Post Preview',
    url: uri.preview,
    method: POST,
    data: {
      switchStatus,
      stream : 0
    }
  }
  return ajax(args)
}

export function sendGetVam (data = {}) {
  let args = {
    methodName: 'Get Vam',
    url: uri.vam,
    method: GET,
    data,
  }
  return ajax(args)
}

export function sendPostVam (data) {
  let args = {
    methodName: 'Post Vam',
    url: uri.vam,
    method: POST,
    data,
  }
  return ajax(args)
}

export function sendPostOpenFaceRecognition (data) {
  let args = {
    methodName: 'Post Open Face Recognition',
    url: `${uri.vamconfig}?type=FR`,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded'
    }
  }
  return ajax(args)
}

export function sendPostCloseFaceRecognition (data) {
  let args = {
    methodName: 'Post Close Face Recognition',
    url: `${uri.vamremoveconfig}?type=FR`,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded'
    }
  }
  return ajax(args)
}

export function sendPostOpenMotionDetection (data) {
  let args = {
    methodName: 'Post Open Motion Detection',
    url: `${uri.vamconfig}?type=MD`,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded'
    }
  }
  return ajax(args)
}

export function sendPostCloseMotionDetection (data) {
  let args = {
    methodName: 'Post Close Motion Detection',
    url: `${uri.vamremoveconfig}?type=MD`,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded'
    }
  }
  return ajax(args)
}

export function sendPostOpenCameraTamper (data) {
  let args = {
    methodName: 'Post Open Camera Tamper',
    url: `${uri.vamconfig}?type=CT`,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded'
    }
  }
  return ajax(args)
}

export function sendPostCloseCameraTamper (data) {
  let args = {
    methodName: 'Post Close Camera Tamper',
    url: `${uri.vamremoveconfig}?type=CT`,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded'
    }
  }
  return ajax(args)
}

export function sendPostFaceRegister (data) {
  let args = {
    methodName: 'Post Face Register',
    url: uri.vamenroll,
    method: POST,
    data,
    headers: {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/x-www-form-urlencoded',
      'Connection': 'keep-alive'
    }
  }
  return ajax(args)
}

export function sendPostTakingPictures () {
  let args = {
    methodName: 'Post Capture Image',
    url: uri.takingPictures,
    method: POST,
    data: {}
  }
  return ajax(args)
}

export function sendGetRecording () {
  let args = {
    methodName: 'Get Recording',
    url: uri.recording,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostRecording (switchStatus) {
  let args = {
    methodName: 'Post Recording',
    url: uri.recording,
    method: POST,
    data: {
      switchStatus
    }
  }
  return ajax(args)
}

export function sendGetNetwork () {
  let args = {
    methodName: 'Get Network',
    url: uri.network,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostNetwork (data) {
  let args = {
    methodName: 'Post Network',
    url: uri.network,
    method: POST,
    data,
    timeout: 1500
  }
  return ajax(args)
}

export function sendGetWifi () {
  let args = {
    methodName: 'Get Wifi',
    url: uri.wifi,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostWifi (data) {
  let args = {
    methodName: 'Post Wifi',
    url: uri.wifi,
    method: POST,
    data,
    timeout: 1500
  }
  return ajax(args)
}

export function sendGetReplay (data) {
  let args = {
    methodName: 'Get Replay',
    url: uri.replay,
    method: GET,
    data
  }
  return ajax(args)
}

export function sendGetImage (data) {
  let args = {
    methodName: 'Get Image',
    url: uri.images,
    method: GET,
    data
  }
  return ajax(args)
}

export function sendDelImage (data) {
  let args = {
    methodName: 'Delete Image',
    url: uri.imagedelete,
    method: POST,
    data
  }
  return ajax(args)
}

export function sendDelVideo (data) {
  let args = {
    methodName: 'Delete Video',
    url: uri.videodelete,
    method: POST,
    data
  }
  return ajax(args)
}

export function sendGetOverlay () {
  let args = {
    methodName: 'Get Overlay',
    url: uri.overlay,
    method: GET,
    data: {}
  }
  return ajax(args)
}

export function sendPostOverlay (data) {
  let args = {
    methodName: 'Post Overlay',
    url: uri.overlay,
    method: POST,
    data
  }
  return ajax(args)
}

export function sendGetOverlayConfig (data) {
  let args = {
    methodName: 'Get Overlay Config',
    url: uri.overlayconfig,
    method: GET,
    data: data ? data : {}
  }
  return ajax(args)
}

export function sendPostOverlayConfig (data) {
  let args = {
    methodName: 'Post Overlay Config',
    url: uri.overlayconfig,
    method: POST,
    data
  }
  return ajax(args)
}

export function sendGetOnvif (data) {
  let args = {
    methodName: 'Get Onvif',
    url: uri.onvif,
    method: GET,
    data
  }
  return ajax(args)
}

export function sendPostOnvif (data) {
  let args = {
    methodName: 'Post Onvif',
    url: uri.onvif,
    method: POST,
    data
  }
  return ajax(args)
}
