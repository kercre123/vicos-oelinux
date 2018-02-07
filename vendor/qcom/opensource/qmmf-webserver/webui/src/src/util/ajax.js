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

import { HOST, DEADLINE } from 'src/settings'
import debug from './debug'
import 'whatwg-fetch'
import Promise from 'bluebird'

function objectToQueryString (obj) {
  let ret = '?'
  let first = true
  for (let variable in obj) {
    if (obj.hasOwnProperty(variable)) {
      ret += (first ? '' : '&') + variable + '=' + obj[variable]
      if (first) {
        first = false
      }
    }
  }

  return ret == '?' ? '' : ret
}

function checkStatus (response) {
  if (response.status >= 200 && response.status < 300) {
    return response
  } else {
    let error = new Error(response.statusText)
    error.status = response.status
    error.response = response
    throw error
  }
}

export default function ajax ({ methodName, url, method, data, headers, timeout }) {
  let input = HOST + '/' + url
  let options = {
    method,
    headers: headers ? headers : {
      'Accept': 'application/json, text/plain, */*',
      'Content-Type': 'application/json'
    },
    credentials: 'same-origin'
  }
  if (method.toLowerCase() === 'get') {
    input += objectToQueryString(data)
  } else if (method.toLowerCase() === 'post' && !headers) {
    options.body = JSON.stringify(data)
  } else if (headers) {
    options.body = data
  }
  let deadline = timeout ? timeout : DEADLINE
  let _timeout = null

  if (methodName === 'Post Face Register') {
    debug(method.toUpperCase(), methodName)
  } else {
    debug(method.toUpperCase(), methodName, 'ARGS', JSON.stringify(data))
  }
  return Promise.race([
    fetch(input, options).then(checkStatus),
    new Promise((resolve, reject) => {
      _timeout = setTimeout(() => reject(new Error('Request Timeout')), deadline)
    })
  ])
  .then(response => {
    clearTimeout(_timeout)
    _timeout = null

    return response.json()
  })
  .then(data => {
    if (data.Data) {
      debug(method.toUpperCase(), methodName, 'SUCCESS')
    } else {
      debug(method.toUpperCase(), methodName, 'SUCCESS', JSON.stringify(data))
    }
    if (!data.status && data.reason) {
      let err = new Error(data.reason)
      err.type = 'reason'
      throw err
    }
    return data
  })
  .catch(error => {
    clearTimeout(_timeout)
    _timeout = null
    if (error.status == 404) {
      let err = new Error()
      err.deal = true
      return Promise.reject(err)
    } else if (error.type === 'reason') {
      layer.msg(error.message, { icon: 5 })
      let err = new Error()
      err.deal = true
      return Promise.reject(err)
    } else {
      return Promise.reject(error)
    }
  })
}
