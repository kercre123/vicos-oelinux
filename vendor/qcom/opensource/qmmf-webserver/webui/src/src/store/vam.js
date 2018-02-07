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

import { sendGetVam, sendPostVam, sendPostVamConfig } from 'src/ajax'
import { openLive, closeLive } from './live'
import { WSHOST } from 'src/settings'

export const REQUEST_VAM = 'REQUEST_VAM'

export const RECEIVE_VAM = 'RECEIVE_VAM'

export const POST_REQUEST_VAM = 'POST_REQUEST_VAM'

export const POST_RECEIVE_VAM = 'POST_RECEIVE_VAM'

export const CHANGE_VAM_STATUS = 'CHANGE_VAM_STATUS'

export const POST_REQUEST_VAM_CONFIG = 'POST_REQUEST_VAM_CONFIG'

export const POST_RECEIVE_VAM_CONFIG = 'POST_RECEIVE_VAM_CONFIG'

export const VAM_COUNTER = 'VAM_COUNTER'

export const CHANGE_VAM_LIVE_STATUS = 'CHANGE_VAM_LIVE_STATUS'

export function requestVam () {
  return {
    type: REQUEST_VAM
  }
}

export function receiveVam (Vam) {
  return {
    type: RECEIVE_VAM,
    payload: Vam
  }
}

export function postRequestVam () {
  return {
    type: POST_REQUEST_VAM
  }
}

export function postReceiveVam () {
  return {
    type: POST_RECEIVE_VAM
  }
}

export function postRequestVamConfig () {
  return {
    type: POST_REQUEST_VAM_CONFIG
  }
}

export function postReceiveVamConfig () {
  return {
    type: POST_RECEIVE_VAM_CONFIG
  }
}

export function changeVamStatus (status) {
  return {
    type: CHANGE_VAM_STATUS,
    payload: status
  }
}

export function getVam (vamconfig) {
  return dispatch => {
    dispatch(requestVam())
    return sendGetVam().then(data => {
      dispatch(receiveVam(data))
      dispatch(changeVamStatus(data.status))
    })
  }
}

export function openVam (vamconfig) {
  let reqData = {
    switchStatus: true
  }
  if (vamconfig) {
    reqData.vamconfig = vamconfig
  }
  return dispatch => {
    dispatch(postRequestVam())
    return sendPostVam(reqData).then(data => {
      if (data.status) {
        dispatch(postReceiveVam())
        dispatch(requestVam())
        return sendGetVam(reqData)
      }
    }).then(data => {
      if (data.status) {
        if (reqData.vamconfig) {
          data.vamconfig = reqData.vamconfig
        }
        dispatch(receiveVam(data))
        dispatch(changeVamStatus(reqData.switchStatus))
      }
    })
  }
}

export function closeVam (vamconfig) {
  let reqData = {
    switchStatus: false,
  }
  if (vamconfig) {
    reqData.vamconfig = vamconfig
  }
  return dispatch => {
    dispatch(postRequestVam())
    return sendPostVam(reqData).then(data => {
      dispatch(postReceiveVam(data))
      dispatch(changeVamStatus(reqData.switchStatus))
    })
  }
}

export function setVam (switchStatus) {
  return dispatch => {
    dispatch(postRequestVam())
    return sendPostVam(switchStatus).then(data => {
      dispatch(postReceiveVam(data))
      if (!switchStatus) {
        dispatch(changeVamStatus(switchStatus))
      }
    })
  }
}

export function changeVamCounter (number) {
  console.log('changeVamCounter: ' + number)
  return {
    type: VAM_COUNTER,
    payload: number,
  }
}

export function changeVamLiveStatus (status) {
  return {
    type: CHANGE_VAM_LIVE_STATUS,
    payload: status,
  }
}

const initialState = {
  isFetching: false,
  isActive: false,
  vamStatus: false,
  rtspUrl: '',
  ctUrl: null,
  mdUrl: null,
  frUrl: null,
  counter: 0,
  vamLiveStatus: false,
}

export default function vamReducer (state = initialState, action) {
  switch (action.type) {
    case REQUEST_VAM:
    case POST_REQUEST_VAM:
      return Object.assign({}, state, { isFetching: true })
      break
    case RECEIVE_VAM:
      let { url, ct_port, md_port, fr_port, proxyport, vamconfig } = action.payload
      const rtspUrl = url ? url.replace(/\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}/, window.location.hostname) : ''
      if (proxyport) {
        switch (vamconfig) {
          case 'MD':
            md_port = proxyport
            break
          case 'FR':
            fr_port = proxyport
            break
          case 'CT':
            ct_port = proxyport
            break
          default:
            break
        }
      }
      return Object.assign({}, state, {
        isFetching: false,
        rtspUrl,
        ctUrl: ct_port ? `${WSHOST}:${ct_port}/ws` : null,
        mdUrl: md_port ? `${WSHOST}:${md_port}/ws` : null,
        frUrl: fr_port ? `${WSHOST}:${fr_port}/ws` : null,
        isActive: true
      })
      break
    case POST_RECEIVE_VAM:
    case POST_RECEIVE_VAM_CONFIG:
      return Object.assign({}, state, { isFetching: false })
      break
    case CHANGE_VAM_STATUS:
      return Object.assign({}, state, { vamStatus: action.payload })
      break
    case VAM_COUNTER:
      return Object.assign({}, state, { counter: state.counter + action.payload })
      break
    case CHANGE_VAM_LIVE_STATUS:
      return Object.assign({}, state, { vamLiveStatus: action.payload })
      break
    default:
      return state
    break
  }
}
