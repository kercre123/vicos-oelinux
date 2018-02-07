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

import { sendGetPreview, sendPostPreview } from 'src/ajax'
import { WSHOST } from 'src/settings'

export const REQUEST_GET_PREVIEW = 'REQUEST_GET_PREVIEW'

export const RECEIVE_GET_PREVIEW = 'RECEIVE_GET_PREVIEW'

export const REQUEST_POST_PREVIEW = 'REQUEST_POST_PREVIEW'

export const RECEIVE_POST_PREVIEW = 'RECEIVE_POST_PREVIEW'

export function requestPreview () {
  return {
    type: REQUEST_GET_PREVIEW
  }
}

export function receivePreview (Preview) {
  return {
    type: RECEIVE_GET_PREVIEW,
    payload: Preview
  }
}

export function postRequestPreview () {
  return {
    type: REQUEST_POST_PREVIEW
  }
}

export function postReceivePreview () {
  return {
    type: RECEIVE_POST_PREVIEW
  }
}

export function getPreview () {
  return (dispatch, getState) => {
    dispatch(requestPreview())
    return sendGetPreview().then(data => {
      dispatch(receivePreview(data))
    })
  }
}

export function setPreview (switchStatus) {
  return dispatch => {
    dispatch(postRequestPreview())
    return sendPostPreview(switchStatus).then(data => {
      dispatch(postReceivePreview(data))
    })
  }
}

const initialState = {
  isFetching: false,
  status: false,
  rtspUrl: '',
  proxyUrl: '',
}

export default function previewReducer (state = initialState, action) {
  switch (action.type) {
    case REQUEST_GET_PREVIEW:
    case REQUEST_POST_PREVIEW:
      return Object.assign({}, state, { isFetching: true })
      break
    case RECEIVE_GET_PREVIEW:
      const { status, url, proxyport } = action.payload
      const rtspUrl = url ? url.replace(/\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}/, window.location.hostname) : ''
      const proxyUrl = proxyport ? `${WSHOST}:${proxyport}/ws` : ''
      return Object.assign({}, state, {
        isFetching: false,
        rtspUrl,
        proxyUrl,
        status
      })
      break
    case RECEIVE_POST_PREVIEW:
      return Object.assign({}, state, { isFetching: false })
      break
    default:
      return state
    break
  }
}
