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

import { sendPostOpenCameraTamper, sendPostCloseCameraTamper } from 'src/ajax'
import { LOCAL_STORE_KEY } from 'src/settings'
import { getExistDataByKey, setExistDataByKey } from 'src/util'

export const POST_REQUEST_CAMERA_TAMPER = 'POST_REQUEST_CAMERA_TAMPER'

export const POST_RECEIVE_CAMERA_TAMPER = 'POST_RECEIVE_CAMERA_TAMPER'

export const CHANGE_CAMERA_TAMPER_STATUS = 'CHANGE_CAMERA_TAMPER_STATUS'

export const CHANGE_CAMERA_TAMPER_LIVE_STATUS = 'CHANGE_CAMERA_TAMPER_LIVE_STATUS'
let ctArSensitivity = getExistDataByKey(LOCAL_STORE_KEY.ct_config)

const cameraTamperConfig = {
  'version': '1.0',
  'id': '58fe0955-5d8e-4bff-b8e0-2916bd95a3be',
  'atomic_rules':[
    {
      'id':'74f780c2-bf6c-4e9e-9465-fd660c53ba75',
      'event_type': 1,
      'name': 'Ipc tamper rule',
      'status': 1,
      'sensitivity': ctArSensitivity ? ctArSensitivity.atomic_rules[0].sensitivity : 1,
    }
  ]
}

const reqData = `camera_id=0&vam_config=${JSON.stringify(cameraTamperConfig)}`

export function postRequestCameraTamper () {
  return {
    type: POST_REQUEST_CAMERA_TAMPER
  }
}

export function postReceiveCameraTamper (data) {
  return {
    type: POST_RECEIVE_CAMERA_TAMPER,
    payload: data
  }
}

export function changeCameraTamperStatus (status) {
  return {
    type: CHANGE_CAMERA_TAMPER_STATUS,
    payload: status
  }
}

export function openCameraTamper () {
  return dispatch => {
    dispatch(postRequestCameraTamper())
    return sendPostOpenCameraTamper(reqData).then(data => {
      dispatch(postReceiveCameraTamper(data))
      if (data.status) {
        dispatch(changeCameraTamperStatus(true))
      }
    })
  }
}

export function closeCameraTamper () {
  return dispatch => {
    dispatch(postRequestCameraTamper())
    return sendPostCloseCameraTamper(reqData).then(data => {
      dispatch(postReceiveCameraTamper(data))
      if (data.status) {
        dispatch(changeCameraTamperStatus(false))
      }
    })
  }
}

export function changeCameraTamperLiveStatus(status) {
  console.log('changeCameraTamperLiveStatus')
  console.log(status)
  return {
    type: CHANGE_CAMERA_TAMPER_LIVE_STATUS,
    payload: status
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  cameraTamperStatus: false,
  cameraTamperLiveStatus: false,
}

export default function cameraTamperReducer (state = initialState, action) {
  switch (action.type) {
    case POST_REQUEST_CAMERA_TAMPER:
      return Object.assign({}, state, { isFetching: true })
      break
    case POST_RECEIVE_CAMERA_TAMPER:
      return Object.assign({}, state, { isFetching: false, isActive: true })
      break
    case CHANGE_CAMERA_TAMPER_STATUS:
      return Object.assign({}, state, { cameraTamperStatus: action.payload })
      break
    case CHANGE_CAMERA_TAMPER_LIVE_STATUS:
      return Object.assign({}, state, { cameraTamperLiveStatus: action.payload })
    default:
      return state
      break
  }
}
