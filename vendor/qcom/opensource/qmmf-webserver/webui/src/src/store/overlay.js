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

import {
  sendGetOverlay, sendPostOverlay,
  sendGetOverlayConfig, sendPostOverlayConfig
} from 'ajax'

export const REQUEST_OVERLAY = 'REQUEST_OVERLAY'

export const RECEIVE_OVERLAY = 'RECEIVE_OVERLAY'

export const POST_REQUEST_OVERLAY = 'POST_REQUEST_OVERLAY'

export const POST_RECEIVE_OVERLAY = 'POST_RECEIVE_OVERLAY'

export function requestOverlay () {
  return {
    type: REQUEST_OVERLAY
  }
}

export function receiveOverlay (overlay) {
  return {
    type: RECEIVE_OVERLAY,
    payload: overlay
  }
}

export function postRequestOverlay () {
  return {
    type: POST_REQUEST_OVERLAY
  }
}

export function postReceiveOverlay () {
  return {
    type: POST_RECEIVE_OVERLAY
  }
}

export function setOverlayConfig (reqData) {
  return (dispatch, getState) => {
    dispatch(requestOverlay())
    return sendPostOverlayConfig(reqData).then(data => {
      dispatch(receiveOverlay(data))
      return data.status
    })
  }
}

export function getOverlayConfig (reqData) {
  return (dispatch, getState) => {
    return sendGetOverlayConfig(reqData).then(data => {
      dispatch(receiveOverlay(data))
    })
  }
}

export function getOverlay () {
  return dispatch => {
    dispatch(requestOverlay())
    return sendGetOverlay().then(data => {
      dispatch(receiveOverlay(data))
    })
  }
}

export function openOverlay () {
  let reqData = { switchStatus: true }
  return dispatch => {
    dispatch(requestOverlay())
    return sendPostOverlay(reqData).then(data => {
      if (data.status) {
        dispatch(receiveOverlay({ switchStatus: true, status: true }))
      } else {
        dispatch(receiveOverlay({ switchStatus: false, status: false }))
      }
    })
  }
}

export function closeOverlay () {
  let reqData = { switchStatus: false }
  return dispatch => {
    dispatch(requestOverlay())
    return sendPostOverlay(reqData).then(data => {
      if (data.status) {
        dispatch(receiveOverlay({ switchStatus: false, status: true }))
      } else {
        dispatch(receiveOverlay({ switchStatus: true, status: false }))
      }
    })
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  ov_type_SelectVal: 0,
  ov_type: [],
  ov_position_SelectVal: 0,
  ov_position: ['topleft', 'topright', 'center', 'bottomleft', 'bottomright', 'none'],
  ov_color: 0,
  ov_usertext: '',
  ov_date_SelectVal: 0,
  ov_date: ['yyyymmdd', 'mmddyyyy'],
  ov_time_SelectVal: 0,
  ov_time: ['hhmmss_24hr', 'hhmmss_ampm', 'hhmm_24hr', 'hhmm_ampm'],
  ov_box_name: '',
  ov_start_x: 0,
  ov_start_y: 0,
  ov_width: 0,
  ov_height: 0,
  ov_image_location: '',
  switchStatus: false,
  status: false
}

export default function overlayReducer (state = initialState, action) {
  switch (action.type) {
    case RECEIVE_OVERLAY:
      return Object.assign({}, state, { ...action.payload, isActive: true })
    default:
      return state
  }
}
