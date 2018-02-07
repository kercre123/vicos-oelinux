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

import { sendGetRecording, sendPostRecording } from 'src/ajax'
import { WSHOST } from 'src/settings'

export const GET_REQUEST_RECORDING = 'GET_REQUEST_RECORDING'

export const GET_RECEIVE_RECORDING = 'GET_RECEIVE_RECORDING'

export const POST_REQUEST_RECORDING = 'POST_REQUEST_RECORDING'

export const POST_RECEIVE_RECORDING = 'POST_RECEIVE_RECORDING'

export const CHANGE_RECORDING_STATUS = 'CHANGE_RECORDING_STATUS'

export const CHANGE_PlAY_STATUS = 'CHANGE_PlAY_STATUS'

export function getRequestRecording () {
  return {
    type: GET_REQUEST_RECORDING
  }
}

export function getReceiveRecording (data) {
  return {
    type: GET_RECEIVE_RECORDING,
    payload: data
  }
}

export function postRequestRecording () {
  return {
    type: POST_REQUEST_RECORDING
  }
}

export function postReceiveRecording (data) {
  return {
    type: POST_RECEIVE_RECORDING,
    payload: data
  }
}

export function changeRecordStatus (status) {
  return {
    type: CHANGE_RECORDING_STATUS,
    payload: status
  }
}
export function postStopPlay (status) {
  return {
    type: CHANGE_PlAY_STATUS,
    payload: status
  }
}
export function getRecording () {
  return dispatch => {
    dispatch(getRequestRecording())
    return sendGetRecording().then(data => {
      dispatch(getReceiveRecording(data))
    })
  }
}

export function startRecording () {
  const status = true
  return dispatch => {
    dispatch(postRequestRecording())
    return sendPostRecording(status).then(data => {
      if (data.status) {
        dispatch(postReceiveRecording())
        dispatch(changeRecordStatus(status))
      }
    })
  }
}
export function stopPlay(){
  return dispatch => {
    return dispatch(postStopPlay(true))
  }
}
export function stopRecording () {
  const status = false
  return dispatch => {
    dispatch(postRequestRecording())
    return sendPostRecording(status).then(data => {
      if (data.status) {
        dispatch(postReceiveRecording())
        dispatch(changeRecordStatus(status))
      }
    })
  }
}

const initialState = {
  isFecthing: false,
  isActive: false,
  recordingStatus: false,
  playStatus:0,
  rtspUrl:'',
  proxyUrl:'',
  replay:0
}

export default function recordingReducer (state = initialState, action) {
  switch (action.type) {
    case GET_REQUEST_RECORDING:
    case POST_REQUEST_RECORDING:
      return Object.assign({}, state, { isFetching: true })
      break
    case POST_RECEIVE_RECORDING:
      return Object.assign({}, state, { isFetching: false })
      break
    case CHANGE_RECORDING_STATUS:
      return Object.assign({}, state, { recordingStatus: action.payload })
    case GET_RECEIVE_RECORDING:
      let { status, url, proxyport } = action.payload
      let replay = 0
      const rtspUrl = url ? url.replace(/\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}/, window.location.hostname) : ''
      const proxyUrl = proxyport ? `${WSHOST}:${proxyport}/ws` : ''
      if(!rtspUrl && (status === false || status === true) ){
        window.sessionStorage.removeItem("rtspUrl")
      }
      else if(rtspUrl && window.sessionStorage.getItem("rtspUrl")){
        status = false
        replay = 1
        window.sessionStorage.removeItem("rtspUrl")
      }
      else if(rtspUrl && !window.sessionStorage.getItem("rtspUrl")){
        replay = 2
        window.sessionStorage.setItem('rtspUrl', rtspUrl)
      }
      return Object.assign({}, state, {recordingStatus: status, isFetching: false, rtspUrl, proxyUrl, replay})
    case CHANGE_PlAY_STATUS:
      const videoStatus = action.payload
      let  playStatus = 0
      if( videoStatus === true && window.sessionStorage.getItem("playStatus")){
        playStatus = 1
        window.sessionStorage.removeItem("playStatus")
      }
      else if(videoStatus === true && !window.sessionStorage.getItem("playStatus")){
        playStatus = 2
        window.sessionStorage.setItem('playStatus', 1)
      }
      return Object.assign({}, state, { playStatus: playStatus })
      break
    default:
      return state
      break
  }
}
