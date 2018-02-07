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

import { sendGetOnvif, sendPostOnvif } from 'src/ajax'

export const REQUEST_ONVIF = 'REQUEST_ONVIF'

export const RECEIVE_ONVIF = 'RECEIVE_ONVIF'

export const POST_REQUEST_ONVIF = 'POST_REQUEST_ONVIF'

export const POST_RECEIVE_ONVIF = 'POST_RECEIVE_ONVIF'

export function requestOnvif () {
    return {
        type: REQUEST_ONVIF
    }
}

export function receiveOnvif (switchStatus) {
    return {
        type: RECEIVE_ONVIF,
        payload: switchStatus
    }
}

export function getOnvif (tag) {
    return dispatch => {
        dispatch(requestOnvif())
        return sendGetOnvif(tag).then(data => {
                dispatch(receiveOnvif(data))
            })
    }
}

export function  postRequestOnvif () {
    return {
        type: POST_REQUEST_ONVIF
    }
}

export function postReceiveOnvif () {
    return {
        type: POST_RECEIVE_ONVIF
    }
}

export function openOnvif () {
    let reqData = { switchStatus: true }
    return dispatch => {
        dispatch(requestOnvif())
        return sendPostOnvif(reqData).then(data => {
            if (data.status) {
                dispatch(receiveOnvif({ switchStatus: true, status: true }))
            } else {
                dispatch(receiveOnvif({ switchStatus: false, status: false }))
            }
        })
    }
}

export function closeOnvif () {
    let reqData = { switchStatus: false }
    return dispatch => {
        dispatch(requestOnvif())
        return sendPostOnvif(reqData).then(data => {
            if (data.status) {
                dispatch(receiveOnvif({ switchStatus: false, status: true }))
            } else {
                dispatch(receiveOnvif({ switchStatus: true, status: false }))
            }
        })
    }
}

const initialState = {
    isFetching: false,
    isActive: false,
    switchStatus: false,
    promptState: false
}

export default function onvifReducer (state = initialState, action) {
    switch (action.type) {
        case REQUEST_ONVIF:
        case POST_REQUEST_ONVIF:
            return Object.assign({}, state, { isFetching: true })
            break
        case RECEIVE_ONVIF:
            let switchStatus = action.payload.switchStatus
            let init_status = window.localStorage.getItem("init_status")
            let promptState = false
            if(switchStatus && init_status == 1){
                window.localStorage.setItem("init_status",0)
                promptState = true
            }
            return Object.assign({}, state, { isFetching: false, switchStatus: switchStatus ,promptState: promptState })
            break
        case POST_RECEIVE_ONVIF:
            return Object.assign({}, state, { isFetching: false })
            break
        default:
            return state
        break
    }
}