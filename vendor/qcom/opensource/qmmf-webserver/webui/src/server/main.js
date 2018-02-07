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

const path = require('path')
const express = require('express')
const bodyParser = require('body-parser')
const webpack = require('webpack')
const webpackDevMiddleware = require('webpack-dev-middleware')
const webpackHotMiddleware = require('webpack-hot-middleware')
const config = require('../config')

const __DEV__ = config.global.__DEV__
const app = express()

app.use(bodyParser.json())

function isEmptyObject (e) {
  let t
  for (t in e)
    return !1
  return !0
}

app.get('/client', function (req, res, next) {
  const fs = require('fs')
  const path = require('path')
  let filePath = path.join(process.cwd(), 'server/client.html')
  const html = fs.readFileSync(filePath)
  res.type('html')
  res.send(html)
})

app.get('/test', function (req, res, next) {
  let ret = isEmptyObject(req.query) ? {html: 'none'} : req.query
  res.json(ret)
})

app.post('/test', function (req, res, next) {
  let ret = isEmptyObject(req.body) ? {html: 'none'} : req.body
  res.json(ret)
})
app.use('/timeout', function (req, res, next) {

})

app.use('/404', function (req, res, next) {
  res.status(404).send('Not Found')
})

app.post('/login', function (req, res, next) {
  let ret = isEmptyObject(req.body) ? {html: 'none'} : req.body
  res.cookie('admin', '123456')
  res.json({status: true})
})

app.post('/logout', function (req, res, next) {
  res.json({status: true})
})

let channels = {
  channel: ["Channel One", "Channel Two", "Channel Three"],
  channelSelectVal: 1,
  status: true
}

app.get('/channel', function (req, res, next) {
  res.json(channels)
})

app.post('/channel', function (req, res, next) {
  const { channelSelectVal } = req.body
  channels.channelSelectVal = channelSelectVal
  res.json({ status: true })
})

let params = {
  SHDR: 'off',
  TNR: 'off',
  status: true
}

app.get('/params', function (req, res, next) {
  res.json(params)
})

app.post('/params', function (req, res, next) {
  const par = req.body
  params = Object.assign({}, params, par)
  res.json({ status: true })
})

let videoSetting = {
  resolutionSelectVal: 1,
  resolution: ["4K", "1080P"],
  encodeModeSelectVal: 1,
  encodeMode: ["HEVC", "AVC"],
  bitRateSelectVal: 4,
  bitRate: [512000, 1000000, 2000000, 4000000, 6000000, 8000000, 10000000, 20000000],
  fpsSelectVal: 2,
  fps: [5, 24, 30],
  status: true
}

app.get('/video', function (req, res, next) {
  res.json(videoSetting)
})

app.post('/video', function (req, res, next) {
  const {
    resolutionSelectVal,
    encodeModeSelectVal,
    bitRateSelectVal,
    fpsSelectVal
  } = req.body

  videoSetting.resolutionSelectVal = resolutionSelectVal
  videoSetting.encodeModeSelectVal = encodeModeSelectVal
  videoSetting.bitRateSelectVal = bitRateSelectVal
  videoSetting.fpsSelectVal = fpsSelectVal

  res.json({ status: true })
})

let previewStatus = false

app.get('/preview', function (req, res, next) {
  let ret = {}
  if (previewStatus) {
    ret = {
      "url": "rtsp://192.168.1.111:8900/live",
      "proxyport": '8080',
      "status": true
    }
  } else {
    ret = {
      status: false
    }
  }

  res.json(ret)
})

app.post('/preview', function (req, res, next) {
  previewStatus = !previewStatus
  res.json({ status: true })
})

let vamStatus = false

app.get('/vam', function (req, res, next) {
  let ret = {}
  if (vamStatus) {
    ret = {
      url: 'rtsp://192.168.1.111:8901/live',
      proxyport: '8081',
      status: true
    }
  } else {
    ret = {
      status: false
    }
  }

  res.json(ret)
})

app.post('/vam', function (req, res, next) {
  vamStatus = !vamStatus
  res.json({ status: true })
})

app.post('/vamConfig', function (req, res, next) {
  res.json({ status: true })
})

app.post('/vamremoveconfig', function (req, res, next) {
  res.json({ status: true })
})

app.post('/captureimage', function (req, res, next) {
  res.json({ status: true })
})

let recordingStatus = false

app.post('/recording', function (req, res, next) {
  recordingStatus = !recordingStatus
  res.json({ status: true })
})

app.get('/recording', function (req, res, next) {
  res.json({ status: recordingStatus })
})

let staticNetwork = {
  "modeSelectVal": 1,
  "mode": ["DHCP", "Static"],
  "ip": "192.168.1.111",
  "mask": "255.255.255.0",
  "status": true
}

let DHCPNetwork = {
  "modeSelectVal": 0,
  "mode": ["DHCP", "Static"],
  status: true
}

let networkSelect = 0

app.get('/network', function (req, res, next) {
  if (networkSelect == 0) {
    res.json(DHCPNetwork)
  } else {
    res.json(staticNetwork)
  }
})

app.post('/network', function (req, res, next) {
  networkSelect = req.body.modeSelectVal
  if (req.body.modeSelectVal === 1) {
    staticNetwork.ip = req.body.ip
    staticNetwork.mask = req.body.mask
  }
})

let wifi = {
    "modeSelectVal": 0,
    "mode": ["AP"],
    "ssid": "xxxx",
    "password": "xxxx",
    "status": true
}

app.get('/wifi', function (req, res, next) {
  res.json(wifi)
})

app.post('/wifi', function (req, res, next) {
  wifi.ssid = req.body.ssid
  wifi.password = req.body.password
  res.json({ status: true })
})

app.get('/replay', function (req, res, next) {
  res.json({
    "files":[
      {"url": "http://localhost:3000/test.mp4","filename":"xxxf2f23f2f23f23f23fr2f2f23f23f23f2x.mp4"},
      {"url": "http://localhost:3000/test2.mp4","filename":"xxx1.mp4"},
      {"url": "http://localhost:3000/test3.mp4","filename":"xxx2.mp4"},
      {"url": "http://localhost:3000/tes4t.mp4","filename":"xx33.mp4"},
      {"url": "http://localhost:3000/test5.mp4","filename":"x444.mp4"},
    ],
    "totalPageNum": 10,
    "status": true
  })
})

app.get('/images', function (req, res, next) {
  res.json({
    "files":[
      {"url": "http://localhost:3000/test5.jpg","filename":"x444.jpg"},
      {"url": "http://localhost:3000/test2.jpg","filename":"x442.jpg"}
    ],
    "totalPageNum": 10,
    "status": true
  })
})

app.post('/videodelete', function (req, res) {
  res.json({ status: true })
})

app.post('/imagedelete', function (req, res) {
  res.json({ status: true })
})

let overlayconfig = {
    "ov_type_SelectVal": 2,
    "ov_type": ["datetime","usertext","boundingbox","staticimage","privacymask"],
    "ov_position_SelectVal": 0,
    "ov_position": ["topleft","topright","center","bottomleft","bottomright","none"],
    "ov_color": '4207199487',
    "ov_usertext": "xxxx",
    "status": true
}

let overlaySwitchStatus = false

app.get('/overlay', function (req, res) {
  res.json({ switchStatus: overlaySwitchStatus })
})

app.post('/overlay', function (req, res) {
  overlaySwitchStatus = !overlaySwitchStatus
  res.json({ status: true })
})

app.get('/overlayconfig', function (req, res) {
  overlayconfig.ov_type_SelectVal = req.query.ov_type_SelectVal
  res.json(overlayconfig)
})

app.post('/overlayconfig', function (req, res) {
  let config = req.body
  overlayconfig = Object.assign({}, overlayconfig, config)
  res.json({ status: true })
})
let onvifSwitchStatus = false
app.get('/onvif', function (req, res) {
  res.json({switchStatus: onvifSwitchStatus})
})
app.post('/onvif', function (req, res) {
  onvifSwitchStatus = !onvifSwitchStatus
  res.json({ status: true })
})

app.use(require('connect-history-api-fallback')())

app.use(express.static(path.join(__dirname, '../public')))

if (__DEV__) {
  const webpackConfig = require('../build/webpack.config')

  const compiler = webpack(webpackConfig)

  app.use(webpackDevMiddleware(compiler, {
    publicPath: webpackConfig.output.publicPath,
    noInfo: true,
    stats: { colors: true }
  }))

  app.use(webpackHotMiddleware(compiler))
}

module.exports = app
