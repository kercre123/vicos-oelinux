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

import { sendGetVideoSetting, sendPostVideoSetting } from 'src/ajax'

export const REQUEST_VIDEO_SETTING = 'REQUEST_VIDEO_SETTING'

export const RECEIVE_VIDEO_SETTING = 'RECEIVE_VIDEO_SETTING'

export const POST_REQUEST_VIDEO_SETTING = 'POST_REQUEST_VIDEO_SETTING'

export const POST_RECEIVE_VIDEO_SETTING = 'POST_RECEIVE_VIDEO_SETTING'

export function requestVideoSetting () {
  return {
    type: REQUEST_VIDEO_SETTING
  }
}

export function receiveVideoSetting (setting) {
  return {
    type: RECEIVE_VIDEO_SETTING,
    payload: setting
  }
}

export function postRequestVideoSetting () {
  return {
    type: POST_REQUEST_VIDEO_SETTING
  }
}

export function postReceiveVideoSetting (setting) {
  return {
    type: POST_RECEIVE_VIDEO_SETTING,
    payload: setting
  }
}

export function getVideoSetting () {
  return (dispatch, getState) => {
    dispatch(requestVideoSetting())
    return sendGetVideoSetting().then(data => {
      if (data.status) {
        dispatch(receiveVideoSetting(data))
      }
      return data.status
    })
  }
}

export function setVideoSetting (setting) {
  return dispatch => {
    dispatch(postRequestVideoSetting())
    return sendPostVideoSetting(setting).then(data => {
      if (data.status) {
        dispatch(postReceiveVideoSetting(setting))
      }
      return data.status
    })
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  setting: {
    resolution: [],
    resolutionSelected: 0,
    encodeMode: [],
    encodeModeSelected: 0,
    bitRate: [],
    bitRateSelected: 0,
    fps: [],
    fpsSelected: 0
  }
}

export default function videoSettingReducer (state = initialState, action) {
  switch (action.type) {
    case REQUEST_VIDEO_SETTING:
    case POST_REQUEST_VIDEO_SETTING:
      return Object.assign({}, state, { isFetching: true })
    case RECEIVE_VIDEO_SETTING: {
      let {
        resolution, resolutionSelectVal,
        encodeMode, encodeModeSelectVal,
        bitRate, bitRateSelectVal,
        fps, fpsSelectVal
      } = action.payload
      return Object.assign({}, state, {
        isFetching: false,
        setting: {
          resolution,
          resolutionSelected: resolutionSelectVal,
          encodeMode,
          encodeModeSelected: encodeModeSelectVal,
          bitRate,
          bitRateSelected: bitRateSelectVal,
          fps,
          fpsSelected: fpsSelectVal
        },
        isActive: true })
    }
    case POST_RECEIVE_VIDEO_SETTING:
      return Object.assign({}, state, {
        isFetching: false, setting: action.payload
      })
    default:
      return state
  }
}
