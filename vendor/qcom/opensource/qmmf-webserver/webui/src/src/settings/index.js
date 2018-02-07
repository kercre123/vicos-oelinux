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

export const version = 'IPC8053 REST API 1.0.1'

export const NAVS = [
    { path: 'live', name: 'Live view' },
    { path: 'setup', name: 'Setup' },
    { path: 'about', name: 'About' }
]

export const WSHOST = `ws://${window.location.hostname}`

export const HOST = `http://${window.location.host}`

export const DEADLINE = 5000

export const WS_LOOP = 10000

export const uri = {
  ws: `${WSHOST}:1080/async`,
  login: 'login',
  logout: 'logout',
  channel: 'channel',
  preview: 'preview',
  vam: 'vam',
  vamconfig: 'vamconfig',
  vamremoveconfig: 'vamremoveconfig',
  vamenroll: 'vamenroll',
  index: 'index',
  network: 'network',
  wifi: 'wifi',
  video: 'video',
  videodelete: 'videodelete',
  takingPictures: 'captureimage',
  recording: 'recording',
  params: 'params',
  replay: 'replay',
  images: 'images',
  imagedelete: 'imagedelete',
  overlay: 'overlay',
  overlayconfig: 'overlayconfig',
  onvif: 'onvif'
}

export const LOCAL_STORE_KEY = {
  fr_config:'faceRecognitionConfig',
  md_config:'motionDetectionConfig',
  ct_config:'cameraTamperConfig',
  ob_config:'objectBoxConfig',
  md_and_ct_config: 'mdAndCtConfig'
}