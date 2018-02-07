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

import { sendGetChannel, sendPostChannel } from 'src/ajax'

export const REQUEST_CHANNEL = 'REQUEST_CHANNEL'

export const RECEIVE_CHANNEL = 'RECEIVE_CHANNEL'

export const POST_REQUEST_CHANNEL = 'POST_REQUEST_CHANNEL'

export const POST_RECEIVE_CHANNEL = 'POST_RECEIVE_CHANNEL'

export function requestChannel () {
  return {
    type: REQUEST_CHANNEL
  }
}

export function receiveChannel (channel) {
  return {
    type: RECEIVE_CHANNEL,
    payload: channel
  }
}

export function getChannel (tag) {
  return dispatch => {
    dispatch(requestChannel())
    return sendGetChannel(tag).then(data => {
      dispatch(receiveChannel(data))
    })
  }
}

export function postRequestChannel () {
  return {
    type: POST_REQUEST_CHANNEL
  }
}

export function postReceiveChannel () {
  return {
    type: POST_RECEIVE_CHANNEL
  }
}

export function setChannel (channelSelected) {
  return dispatch => {
    dispatch(postRequestChannel())
    return sendPostChannel(channelSelected).then(data => {
      if (data.status) {
        dispatch(postReceiveChannel())
      } else {
        return Promise.reject(new Error('Change Channel Error'))
      }
    })
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  list: [],
  selected: 0
}

export default function channelReducer (state = initialState, action) {
  switch (action.type) {
    case REQUEST_CHANNEL:
    case POST_REQUEST_CHANNEL:
      return Object.assign({}, state, { isFetching: true })
    break
    case RECEIVE_CHANNEL:
      let { channel, channelSelectVal } = action.payload
      return Object.assign({}, state, {
        isFetching: false,
        list: channel, selected: channelSelectVal, isActive: true
      })
    case POST_RECEIVE_CHANNEL:
      return Object.assign({}, state, {
        isFetching:false,
        isActive: true
      })
      break
    default:
      return state
      break
  }
}
