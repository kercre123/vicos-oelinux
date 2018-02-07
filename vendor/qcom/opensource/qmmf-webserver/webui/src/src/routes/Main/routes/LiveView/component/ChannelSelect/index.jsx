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
import { setChannel } from 'store/channel'
import { closeOverlay } from 'store/overlay'
import { closeLive } from 'store/live'
import { setPreview } from 'store/preview'
import { stopRecording } from 'store/recording'
import { changePaddingStatus } from 'store/padding'
import { closeVam } from 'src/store/vam'
import { openFaceRecognition, closeFaceRecognition, changeFaceRecognitionLiveStatus } from 'src/store/faceRecognition'
import { openCameraTamper, closeCameraTamper, changeCameraTamperLiveStatus } from 'src/store/cameraTamper'
import { openMotionDetection, closeMotionDetection, changeMotionDetectionLiveStatus } from 'src/store/motionDetection'
import Promise from 'bluebird'

import './channelSelect.sass'

class ChannelSelect extends Component {
  changeChannel = async e => {
    if (this.props.padding) {
      return
    }
    const value = parseInt(e.target.value, 10)
    const {
      changePaddingStatus,
      overlayStatus, closeOverlay,
      faceRecognitionLiveStatus, changeFaceRecognitionLiveStatus, closeFaceRecognition,
      motionDetectionLiveStatus, changeMotionDetectionLiveStatus, closeMotionDetection,
      cameraTamperLiveStatus, changeCameraTamperLiveStatus, closeCameraTamper,
      closeVam,
      recordingStatus, stopRecording,
      liveStatus, closeLive, setPreview,
      setChannel, changeEnd,
    } = this.props
    try {
      changePaddingStatus(true)
      overlayStatus && await closeOverlay()
      if (faceRecognitionLiveStatus) {
        changeFaceRecognitionLiveStatus(false)
        await new Promise(r => setTimeout(() => r(closeFaceRecognition()), 500))
        await new Promise(r => setTimeout(() => r(closeVam('FR')), 500))
      }
      if (motionDetectionLiveStatus) {
        changeMotionDetectionLiveStatus(false)
        await new Promise(r => setTimeout(() => r(closeMotionDetection()), 500))
        await new Promise(r => setTimeout(() => r(closeVam('MD')), 500))
      }
      if (cameraTamperLiveStatus) {
        changeCameraTamperLiveStatus(false)
        await new Promise(r => setTimeout(() => r(closeCameraTamper()), 500))
        await new Promise(r => setTimeout(() => r(closeVam('CT')), 500))
      }
      recordingStatus && await stopRecording()
      liveStatus && await closeLive().then(() => setPreview(false))
      await setChannel(value)
      await changeEnd()
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
      if( $("#record_time").length > 0 ) $("#record_time").remove()
    } catch (error) {
      !error.deal && console.log(error)
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
    }
  }

  renderOptions = () => {
    return this.props.channel.list.map((channel, i) => {
      return <option key={i} value={i}>{channel}</option>
    })
  }

  render () {
    const { selected } = this.props.channel
    return (
      <div id="channel-select">
        <i id="channel-icon"></i>
        <select onChange={this.changeChannel} value={selected}>
          {this.renderOptions()}
        </select>
      </div>
    )
  }
}

const mapStateToProps = state => ({
  channel: state.channel,
  liveStatus: state.live.liveStatus,
  overlayStatus: state.overlay.switchStatus,
  recordingStatus: state.recording.recordingStatus,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus,
  motionDetectionLiveStatus: state.motionDetection.motionDetectionLiveStatus,
  cameraTamperLiveStatus: state.cameraTamper.cameraTamperLiveStatus,
  padding: state.padding
})

const mapDispatchToProps = {
  setChannel,
  closeLive,
  setPreview,
  closeOverlay,
  closeFaceRecognition,
  changeFaceRecognitionLiveStatus,
  closeCameraTamper,
  changeCameraTamperLiveStatus,
  closeMotionDetection,
  changeMotionDetectionLiveStatus,
  closeVam,
  stopRecording,
  changePaddingStatus,
}

ChannelSelect.propTypes = {
  changeEnd: PropTypes.func.isRequired
}

export default connect(mapStateToProps, mapDispatchToProps)(ChannelSelect)
