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

import { getCookie, debug } from 'src/util'
import { WS_LOOP } from 'src/settings'

export const CREATE_SOCKET = 'CREATE_SOCKET'

export const CHANGE_SOCKET_STRATUS = 'CHANGE_SOCKET_STRATUS'

export const CREATE_KEEP_ALIVE = 'CREATE_KEEP_ALIVE'

export const SEND_FIRST_KEEP_ALIVE = 'SEND_FIRST_KEEP_ALIVE'

export const SEND_INTERVAL_KEEP_ALIVE = 'SEND_INTERVAL_KEEP_ALIVE'

export const CLEAR_SOCKET = 'CLEAR_SOCKET'

export function createSocket (socket) {
  return {
    type: CREATE_SOCKET,
    payload: socket
  }
}

export function changeSocketStatus (status) {
  return {
    type: CHANGE_SOCKET_STRATUS,
    payload: status
  }
}

export function createKeepAlive (obj) {
  return {
    type: CREATE_KEEP_ALIVE,
    payload: obj
  }
}

export function sendFKA () {
  return {
    type: SEND_FIRST_KEEP_ALIVE
  }
}

export function sendIntervalKeepAlive () {
  return {
    type: SEND_INTERVAL_KEEP_ALIVE
  }
}

export function sendFirstKeepAlive (obj) {
  return dispatch => {
    dispatch(createKeepAlive(obj))
    dispatch(sendFKA())
  }
}

export function initSocket (socket) {
  return dispatch => {
    dispatch(createSocket(socket))
    return new Promise((resolve, reject) => {
      socket.onopen = () => {
        dispatch(changeSocketStatus(true))
        resolve('open')
      }
    })
  }
}

export function clearSocket () {
  return {
    type: CLEAR_SOCKET
  }
}

const initialState = {
  connect: null,
  status: false,
  keepAlive: null,
  interval: null
}

export default function socketReducer (state = initialState, action) {
  switch (action.type) {
    case CREATE_SOCKET:
      return Object.assign({}, state, { connect: action.payload })
      break
    case CHANGE_SOCKET_STRATUS:
      return Object.assign({}, state, { status: action.payload })
      break
    case CREATE_KEEP_ALIVE:
      return Object.assign({}, state, { keepAlive: action.payload })
      break
    case SEND_FIRST_KEEP_ALIVE:
      debug('Send First keep-alive ----')
      state.connect.send(JSON.stringify(state.keepAlive))
      return state
      break
    case SEND_INTERVAL_KEEP_ALIVE:
      if (state.interval === null) {
        state.interval = setInterval(() => {
          debug('Send keep-alive', JSON.stringify(state.keepAlive))
          state.connect.send(JSON.stringify(state.keepAlive))
        }, WS_LOOP)
      }
      return state
      break
    case CLEAR_SOCKET:
      if (state.interval !== null) {
        clearInterval(state.interval)
      }
      state.connect.close()
      return Object.assign({}, state, {connect: null, status: false, keepAlive: null, interval: null})
    default:
      return state
      break
  }
}
