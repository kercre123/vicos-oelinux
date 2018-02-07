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

import React, { Component, PropTypes } from 'react'
import { connect } from 'react-redux'
import * as rtsp from 'html5_rtsp_player'
import initVa from 'va'
import layer from 'layerUI'
import { HOST } from 'src/settings'
import { debug } from 'src/util'
import styles from './liveViewArea.sass'
import Canvas from './Canvas'
import { initChromePlayer, initVlc } from './initVideo'
import { registerCanvas, registerVideo } from 'src/store/live'

const TAG = 'Live View Area'

class LiveViewArea extends Component {
  constructor (props) {
    super(props)
  }

  startPlay = () => {
    debug('startPlay')
    if (__DEV__) {
      return
    }
    const { isChrome, liveRtspUrl, liveProxyUrl } = this.props
    return new Promise(resolve => {
      if (isChrome) {
        rtsp.RTSP_CONFIG['websocket.url'] = liveProxyUrl
        this.video.setAttribute('rtsp_url', liveRtspUrl)
        this.player = rtsp.attach(this.video)
        this.player.start()
        var canvas = this.canvas
        this.video.oncanplaythrough = function () {
          this.play()
          canvas.start()
        }
      } else {
        let options = [
          false ? ':audio' : ':no-audio',
          ':network-caching=' + 180
        ]
        try {
          this.vlc.playlist.items.clear()
          this.vlc.playlist.add(liveRtspUrl, '', options)
          this.vlc.playlist.play()
        } catch (e) {
          debug(TAG, 'Play video occure error: ' + e)
          let msg = 'VLC Web Plugin encountered error, please reload page.'
          layer.confirm(msg, {
            icon: 0,
            title: ['alarm', true],
            area: '450px',
            btn: ['OK', 'Cancel']
          }, function () {
            window.location.reload()
          })
        }
      }
    })
  }

  stopPlay = () => {
    debug('stopPlay')
    if (__DEV__) {
      return
    }
    const { isChrome } = this.props
    return new Promise(resolve => {
      if (isChrome) {
        debug('this.player.stop()')
        this.player.stop()
        this.player = null
        this.video.setAttribute('src', '')
        this.canvas.stop()
      } else {
        this.vlc.playlist.stop()
        this.vlc.playlist.items.clear()
      }
      resolve()
    })
  }

  startFaceRecognition = () => {
    debug('startFaceRecognition')
    const { vam: { rtspUrl, frUrl } } = this.props
    let args = {
      url: rtspUrl,
      wsUrl: frUrl,
    }
    if (__DEV__) {
      console.log(args)
      return
    }
    this.canvas.startDrawFr()
    this.stopFr = initVa(
        args,
        this.canvas.onFrMessage.bind(this.canvas),
        this.canvas.clearFr.bind(this.canvas)
    )
  }

  stopFaceRecognition = () => {
    debug('stopFaceRecognitio')
    if (__DEV__) {
      return
    }
    this.canvas.stopDrawFr()
    this.stopFr()
  }

  startMotionDetection = () => {
    debug('startMotionDetection')
    const { vam: { rtspUrl, mdUrl } } = this.props
    let args = {
      url: rtspUrl,
      wsUrl: mdUrl,
    }
    if (__DEV__) {
      console.log(args)
      return
    }

    this.canvas.startDrawMd()
    this.stopMd = initVa(
        args,
        this.canvas.onMdMessage.bind(this.canvas),
        this.canvas.clearMd.bind(this.canvas)
    )
  }

  stopMotionDetection = () => {
    debug('stopMotionDetection')
    if (__DEV__) {
      return
    }
    this.canvas.stopDrawMd()
    this.stopMd()
  }

  startCameraTamper = () => {
    debug('startCameraTamper')
    const { vam: { rtspUrl, ctUrl } } = this.props
    let args = {
      url: rtspUrl,
      wsUrl: ctUrl,
    }
    if (__DEV__) {
      console.log(args)
      return
    }
    this.stopCt = initVa(
        args,
        this.canvas.onCtMessage.bind(this.canvas),
        this.canvas.clearCt.bind(this.canvas)
    )
  }

  stopCameraTamper = () => {
    debug('stopCameraTamper')
    if (__DEV__) {
      return
    }
    this.stopCt()
  }

  renderVideo = () => {
    const { isChrome, hasVlcPlugin } = this.props
    if (isChrome) {
      return this.renderChromeVideo()
    }
    if (!isChrome && hasVlcPlugin) {
      return this.renderFirFoxVideo()
    }
    if (!isChrome && !hasVlcPlugin) {
      return this.renderVlcPlugin()
    }
  }

  renderFirFoxVideo = () => {
    let html = '';
    html += '<embed width="100%" height="100%" id="vlc" ';
    html += 'type="application/x-vlc-plugin" pluginspage="http://www.videolan.org" ';
    html += 'loop="false" autoplay="false" z-index="-1" bgcolor="#000000" ';
    html += 'windowless="true" text="Loading..." version="VideoLAN.VLCPlugin.2">';
    html += '<\/embed>';

    return (
        <div className="bg" dangerouslySetInnerHTML={{ __html: html }}></div>
    )
  }

  renderChromeVideo = () => {
    return (
        <div className="bg">
          <video id="rtsp-player" style={{display: 'none'}}></video>
          <canvas height="540" width="960" id="canvas" style={{position:'relative'}}></canvas>
        </div>
    )
  }

  renderVlcPlugin = () => {
    let href = HOST + '/plugin/vlc_V2.2.1_setup.1430727715.exe'
    return (
        <div style={{paddingTop: '300px', textAlign: 'center'}}>
          <a style={{textDecoration: 'underline'}} href={href} target="_blank">
            There is no VLC plugin,please download it.</a>
        </div>
    )
  }

  componentDidMount () {
    const { isChrome } = this.props
  }
  componentWillReceiveProps(nextProps){
    const {
        isChrome,liveStatus:CLS,
        playStatus:CPS,
        videoSetting: CVS,
        replay: CRP
    } = this.props
    const {
        liveStatus:NLS,
        playStatus:NPS,
        videoSetting: NVS,
        replay: NRP
    } = nextProps

    if (isChrome) {
      if(NPS && CPS !== NPS){
        this.stopPlay()
      }
      if(CVS !== NVS){
        const {resolution ,resolutionSelected} = NVS
        let type = resolution[resolutionSelected]
        try{
          this.video = initChromePlayer(TAG, 'rtsp-player')
          this.canvas = new Canvas('canvas', this.video, type)
          this.props.registerCanvas(this.canvas)
          this.props.registerVideo(this.video)
        } catch(err){
          console.log(err)
        }
      }
      else if (NRP && CRP !== NRP) {
        let type = ""
        if(NRP === 1){
          type =  "4K"
        }
        else if (NRP === 2){
          type = "480P"
        }
        if(type) {
          try{
            this.video = initChromePlayer(TAG, 'rtsp-player')
            this.canvas = new Canvas('canvas', this.video, type)
            this.props.registerCanvas(this.canvas)
            this.props.registerVideo(this.video)
            setTimeout(() => {this.startPlay()}, 200)
          } catch(err){
            console.log(err)
          }
        }
      }
    }else{
      this.vlc = initVlc(TAG, 'vlc')
    }
  }

  shouldComponentUpdate (nextProps) {
    const {
        isChrome, liveStatus: CLS,
        faceRecognitionLiveStatus: CFRS,
        motionDetectionLiveStatus: CMDS,
        cameraTamperLiveStatus: CCTS
    } = this.props
    const {
        liveStatus: NLS,
        faceRecognitionLiveStatus: NFRS,
        motionDetectionLiveStatus: NMDS,
        cameraTamperLiveStatus: NCTS
    } = nextProps
    return CLS !== NLS || CFRS !== NFRS || CMDS !== NMDS || CCTS !== NCTS
  }

  componentWillUpdate (nextProps) {
    const {
        isChrome, liveStatus: CLS,
        faceRecognitionLiveStatus: CFRS,
        motionDetectionLiveStatus: CMDS,
        cameraTamperLiveStatus: CCTS,
    } = this.props
    const {
        liveStatus: NLS,
        faceRecognitionLiveStatus: NFRS,
        motionDetectionLiveStatus: NMDS,
        cameraTamperLiveStatus: NCTS,
    } = nextProps
    if (CLS !== NLS) {
      NLS ? this.startPlay() : this.stopPlay()
    }
    if (CFRS !== NFRS && isChrome) {
      NFRS ? this.startFaceRecognition() : this.stopFaceRecognition()
    }
    if (CMDS !== NMDS && isChrome) {
      NMDS ? this.startMotionDetection() : this.stopMotionDetection()
    }
    if (CCTS !== NCTS && isChrome) {
      NCTS ? this.startCameraTamper() : this.stopCameraTamper()
    }
  }

  componentWillUnmount () {
    const {
        liveStatus,
        faceRecognitionLiveStatus,
        motionDetectionLiveStatus,
        cameraTamperLiveStatus,
    } = this.props
    faceRecognitionLiveStatus && this.stopFaceRecognition()
    motionDetectionLiveStatus && this.stopMotionDetection()
    cameraTamperLiveStatus && this.stopCameraTamper()
    liveStatus && this.stopPlay()
  }

  render () {
    return (
        <div id="live-view-area">
          {this.renderVideo()}
        </div>
    )
  }
}

const mapStateToProps = state => ({
  isChrome: state.live.isChrome,
  hasVlcPlugin: state.live.hasVlcPlugin,
  liveStatus: state.live.liveStatus,
  replay: state.recording.replay,
  playStatus: state.recording.playStatus,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus,
  motionDetectionLiveStatus: state.motionDetection.motionDetectionLiveStatus,
  cameraTamperLiveStatus: state.cameraTamper.cameraTamperLiveStatus,
  liveRtspUrl: state.recording.rtspUrl ? state.recording.rtspUrl : state.preview.rtspUrl,
  liveProxyUrl: state.recording.proxyUrl ? state.recording.proxyUrl : state.preview.proxyUrl,
  vam: state.vam,
  videoSetting:state.video.setting,
  selectChannel:state.channel
})

const mapDispatchToProps = {
  registerCanvas,
  registerVideo
}

export default connect(mapStateToProps, mapDispatchToProps)(LiveViewArea)
