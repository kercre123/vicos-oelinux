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
import { debug } from 'src/util'
import { connect } from 'react-redux'
import { setPreview, getPreview } from 'store/preview'
import { openLive, closeLive } from 'store/live'
import { changePaddingStatus } from 'store/padding'
import layer from 'layerUI'
import classNames from 'classnames'
import './liveViewBtn.sass'

class LiveViewBtn extends Component {
  switchPreview = () => {
    const { padding, liveStatus } = this.props
    if (padding) {
      return
    }
    if (liveStatus) {
      return  setTimeout(() => {this.close()}, 500)
    }
    return setTimeout(() => {this.open()}, 500)
  }

  open = async () => {
    const {
      isChrome,
      videoSettings: { encodeModeSelected, resolutionSelected },
      recordingStatus,
      setPreview,
      getPreview,
      openLive,
      changePaddingStatus,
    } = this.props
    if (isChrome && encodeModeSelected === 0) {
      return layer.msg(
        'Browser is not support HEVC,Please change the video setting',
        { icon: 5 }
      )
    }
    if (recordingStatus) {
      return layer.msg('Currently in recording, cannot open preview', { icon: 5 })
    }
    try {
      changePaddingStatus(true)
      await setPreview(true)
      await getPreview()
      await openLive()
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
    } catch (error) {
      !error.deal && console.log(error)
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
    }
  }

  close = async () => {
    const {
      faceRecognitionLiveStatus,
      recordingStatus,
      motionDetectionLiveStatus,
      cameraTamperLiveStatus,
      overlayStatus,
      closeLive,
      setPreview,
    } = this.props
    if (recordingStatus) {
      return layer.msg('Please turn off the Recording first!', { icon: 5 })
    }
    if (faceRecognitionLiveStatus) {
      return layer.msg('Please turn off the Face Recognition first!', { icon: 5 })
    }
    if (motionDetectionLiveStatus) {
      return layer.msg('Please turn off the Motion Detection first!', { icon: 5 })
    }
    if (cameraTamperLiveStatus) {
      return layer.msg('Please turn off the Camera Tamper first!', { icon: 5 })
    }
    if (overlayStatus) {
      return layer.msg('Please turn off the Overlay first!', { icon: 5 })
    }
    try {
      changePaddingStatus(true)
      await closeLive()
      await setPreview(false)
      await getPreview()
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
    } catch (error) {
      !error.deal && console.log(error)
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
    }
  }

  render () {
    const { liveStatus } = this.props
    let className = classNames({
      open: liveStatus
    })

    return (
      <a id="live-view" className={className} onClick={this.switchPreview}></a>
    )
  }
}

const mapStateToProps = state => ({
  overlayStatus: state.overlay.switchStatus,
  recordingStatus: state.recording.recordingStatus,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus,
  motionDetectionLiveStatus: state.motionDetection.motionDetectionLiveStatus,
  cameraTamperLiveStatus: state.cameraTamper.cameraTamperLiveStatus,
  liveStatus: state.live.liveStatus,
  isChrome: state.live.isChrome,
  videoSettings: state.video.setting,
  padding: state.padding,
  preview: state.preview,
})

const mapDispatchToProps = {
  setPreview,
  getPreview,
  openLive,
  closeLive,
  changePaddingStatus
}

export default connect(mapStateToProps, mapDispatchToProps)(LiveViewBtn)
