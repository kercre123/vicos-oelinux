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

import { sendPostOpenMotionDetection, sendPostCloseMotionDetection } from 'src/ajax'
import { LOCAL_STORE_KEY } from 'src/settings'
import { getExistDataByKey, setExistDataByKey } from 'src/util'

export const POST_REQUEST_MOTION_DETECTION = 'POST_REQUEST_MOTION_DETECTION'

export const POST_RECEIVE_MOTION_DETECTION = 'POST_RECEIVE_MOTION_DETECTION'

export const CHANGE_MOTION_DETECTION_STATUS = 'CHANGE_MOTION_DETECTION_STATUS'

export const CHANGE_MOTION_DETECTION_LIVE_STATUS = 'CHANGE_MOTION_DETECTION_LIVE_STATUS'

let mdArSensitivity = getExistDataByKey(LOCAL_STORE_KEY.md_config)
let obArMinSize = getExistDataByKey(LOCAL_STORE_KEY.ob_config)

const motionDetectionConfig = {
  'version': '1.0',
  'id': '58fe0955-5d8e-4bff-b8e0-2916bd95a3be',
  'atomic_rules':[
    {
      'id':'1389bc80-8e6d-45cf-a769-ca3515800c07',
      'event_type': 10,
      'name': 'Motion detection rule',
      'status': 1,
      'sensitivity': mdArSensitivity ? mdArSensitivity.atomic_rules[0].sensitivity : 70,
      'min_size': obArMinSize ? obArMinSize.atomic_rules[0].min_size : 10,
      'reserve': [500,500]
    }
  ]
}

const reqData = `camera_id=0&vam_config=${JSON.stringify(motionDetectionConfig)}`

export function postRequestMotionDetection () {
  return {
    type: POST_REQUEST_MOTION_DETECTION
  }
}

export function postReceiveMotionDetection (data) {
  return {
    type: POST_RECEIVE_MOTION_DETECTION,
    payload: data
  }
}

export function changeMotionDetectionStatus (status) {
  return {
    type: CHANGE_MOTION_DETECTION_STATUS,
    payload: status
  }
}

export function openMotionDetection () {
  return dispatch => {
    dispatch(postRequestMotionDetection())
    return sendPostOpenMotionDetection(reqData).then(data => {
      dispatch(postReceiveMotionDetection(data))
      if (data.status) {
        dispatch(changeMotionDetectionStatus(true))
      }
    })
  }
}

export function closeMotionDetection () {
  return dispatch => {
    dispatch(postRequestMotionDetection())
    return sendPostCloseMotionDetection(reqData).then(data => {
      dispatch(postReceiveMotionDetection(data))
      if (data.status) {
        dispatch(changeMotionDetectionStatus(false))
      }
    })
  }
}

export function changeMotionDetectionLiveStatus(status) {
  console.log('changeMotionDetectionLiveStatus')
  console.log(status)
  return {
    type: CHANGE_MOTION_DETECTION_LIVE_STATUS,
    payload: status
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  motionDetectionStatus: false,
  motionDetectionLiveStatus: false,
}

export default function motionDetectionReducer (state = initialState, action) {
  switch (action.type) {
    case POST_REQUEST_MOTION_DETECTION:
      return Object.assign({}, state, { isFetching: true })
      break
    case POST_RECEIVE_MOTION_DETECTION:
      return Object.assign({}, state, { isFetching: false, isActive: true })
      break
    case CHANGE_MOTION_DETECTION_STATUS:
      return Object.assign({}, state, { motionDetectionStatus: action.payload })
      break
    case CHANGE_MOTION_DETECTION_LIVE_STATUS:
      return Object.assign({}, state, { motionDetectionLiveStatus: action.payload })
    default:
      return state
      break
  }
}
