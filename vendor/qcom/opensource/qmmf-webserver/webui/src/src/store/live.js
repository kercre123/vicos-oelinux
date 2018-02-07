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

export const CHANGE_LIVE_STATUS = 'CHANGE_LIVE_STATUS'

export const REGISTER_CANVAS = 'REGISTER_CANVAS'

export function registerCanvas (canvas) {
  return {
    type: REGISTER_CANVAS,
    payload: canvas
  }
}

export const REGISTER_VIDEO = 'REGISTER_VIDEO'
export const CHANGE_AUDIO_STATUS = 'CHANGE_AUDIO_STATUS'
export function registerVideo (video) {
  return {
    type: REGISTER_VIDEO,
    payload: video
  }
}

export function changeAudioStatus (status) {
  return {
    type: CHANGE_AUDIO_STATUS,
    payload: status
  }
}

export function changeLiveStatus (status) {
  return {
   type: CHANGE_LIVE_STATUS,
   payload: status
 }
}

export function openLive () {
  console.log('openLive')
  return (dispatch, getState) => {
    if (getState().live.lvieStatus !== true) {
      return new Promise(resolve => {
        setTimeout(() => {
          resolve(dispatch(changeLiveStatus(true)))
        }, 500)
      })
    }
  }
}

export function closeLive () {
  console.log('closeLive')
  return (dispatch, getState) => {
    if (getState().live.liveStatus !== false) {
      return new Promise(resolve => {
        dispatch(changeLiveStatus(false))
        setTimeout(() => {
          resolve()
        }, 500)
      })
    }
  }
}

const initialState = {
  liveStatus: false,
  canvas: null,
  video: null,
  audioStatus: false,
  isChrome: true,
  hasVlcPlugin: true,
}

export default function liveReducer (state = initialState, action) {
  switch (action.type) {
    case CHANGE_LIVE_STATUS:
      return Object.assign({}, state, { liveStatus: action.payload })
    case REGISTER_CANVAS:
      return Object.assign({}, state, { canvas: action.payload })
    case REGISTER_VIDEO:
      return Object.assign({}, state, { video: action.payload })
    case CHANGE_AUDIO_STATUS:
      return Object.assign({}, state, { audioStatus: action.payload })
    default:
      return state
  }
}
