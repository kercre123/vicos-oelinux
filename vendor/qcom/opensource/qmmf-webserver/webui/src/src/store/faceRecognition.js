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

import { sendPostOpenFaceRecognition, sendPostCloseFaceRecognition } from 'src/ajax'
import { LOCAL_STORE_KEY } from 'src/settings'
import { getExistDataByKey, setExistDataByKey } from 'src/util'

export const POST_REQUEST_FACE_RECOGNITION = 'POST_REQUEST_FACE_RECOGNITION'

export const POST_RECEIVE_FACE_RECOGNITION = 'POST_RECEIVE_FACE_RECOGNITION'

export const CHANGE_FACE_RECOGNITION_STATUS = 'CHANGE_FACE_RECOGNITION_STATUS'

export const CHANGE_FACE_RECOGNITION_LIVE_STATUS = 'CHANGE_FACE_RECOGNITION_LIVE_STATUS'
let frArSensitivity = getExistDataByKey(LOCAL_STORE_KEY.fr_config)

const faceRecognitionConfig = {
  'version':'1.0',
  'id':'574b152c-4db2-47c4-9254-56de4e61df07',
  'atomic_rules': [
    {
      'id' :'75f2694c-137b-4f54-9d53-1573f1545410',
      'event_type':8,
      'name':'Face recognition rule',
      'status':1,
      'sensitivity': frArSensitivity ? frArSensitivity.atomic_rules[0].sensitivity : 62
    }
  ]
}
const reqData = `camera_id=0&vam_config=${JSON.stringify(faceRecognitionConfig)}`

export function postRequestFaceRecognition () {
  return {
    type: POST_REQUEST_FACE_RECOGNITION
  }
}

export function postReceiveFaceRecognition (data) {
  return {
    type: POST_RECEIVE_FACE_RECOGNITION,
    payload: data
  }
}

export function changeFaceRecognitionStatus (status) {
  return {
    type: CHANGE_FACE_RECOGNITION_STATUS,
    payload: status
  }
}

export function openFaceRecognition () {
  return dispatch => {
    dispatch(postRequestFaceRecognition())
    return sendPostOpenFaceRecognition(reqData).then(data => {
      dispatch(postReceiveFaceRecognition(data))
      if (data.status) {
        dispatch(changeFaceRecognitionStatus(true))
      }
    })
  }
}

export function closeFaceRecognition () {
  return dispatch => {
    dispatch(postRequestFaceRecognition())
    return sendPostCloseFaceRecognition(reqData).then(data => {
      dispatch(postReceiveFaceRecognition(data))
      if (data.status) {
        dispatch(changeFaceRecognitionStatus(false))
      }
    })
  }
}

export function changeFaceRecognitionLiveStatus(status) {
  console.log('changeFaceRecognitionLiveStatus')
  console.log(status)
  return {
    type: CHANGE_FACE_RECOGNITION_LIVE_STATUS,
    payload: status
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  faceRecognitionStatus: false,
  faceRecognitionLiveStatus: false,
}

export default function faceRecognitionReducer (state = initialState, action) {
  switch (action.type) {
    case POST_REQUEST_FACE_RECOGNITION:
      return Object.assign({}, state, { isFetching: true })
      break
    case POST_RECEIVE_FACE_RECOGNITION:
      return Object.assign({}, state, { isFetching: false, isActive: true })
      break
    case CHANGE_FACE_RECOGNITION_STATUS:
      return Object.assign({}, state, { faceRecognitionStatus: action.payload })
      break
    case CHANGE_FACE_RECOGNITION_LIVE_STATUS:
      return Object.assign({}, state, { faceRecognitionLiveStatus: action.payload })
    default:
      return state
      break
  }
}
