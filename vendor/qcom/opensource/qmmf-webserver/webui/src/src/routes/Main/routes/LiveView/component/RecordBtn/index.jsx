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
import { startRecording, stopRecording, getRecording, stopPlay } from 'store/recording'
import { closeOverlay } from 'store/overlay'
import { changePaddingStatus } from 'store/padding'
import classNames from 'classnames'
import './recordBtn.sass'
import layer from 'layerUI'

class RecordBtn extends Component {
  onClick = () => {
    const { padding, recordingStatus } = this.props
    if (padding) {
      return
    }
    if (recordingStatus) {
      return this.close()
    }
    return this.open()
  }

  open = async () => {
    const {
        liveStatus,
        startRecording,
        getRecording,
        changePaddingStatus,
        recordingStatus,
        videoSetting,
        stopPlay
    } = this.props
    if (!liveStatus) {
      return layer.msg('Please open preview first', { icon: 5 })
    }
    try {
      changePaddingStatus(true)
      const {resolution, resolutionSelected } = videoSetting
      let type = resolution[resolutionSelected]

      if(type == "4K")  await new Promise(r => setTimeout(() => r(stopPlay(true)), 200))
      await startRecording()
      await getRecording()

      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
      if (this.props.recordingStatus) {
        this.startClock()
        layer.msg('Open videotape Success!', { icon: 6 })
      } else {
        layer.msg('Open videotape Fail！Please try again later.', { icon: 5 })
      }
    } catch (error) {
      !error.deal && console.log(error)
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
      layer.msg('Open videotape Fail！Please try again later.', { icon: 5 })
    }
  }

  startClock = () => {
    this.stopClock()
    let record_time_obj = document.getElementById('record_time')
    if(!record_time_obj){
      let canvas = document.createElement('canvas')
      canvas.id= "record_time"
      $(".bar-left").append(canvas)
      record_time_obj = document.getElementById('record_time')
    }
    this.canvas = record_time_obj
    this.canvas.width = 80
    this.canvas.height = 24
    this.context = this.canvas.getContext('2d')
    this.context.font = '16px/1 arial'
    this.context.textAlign = "center"
    this.context.fillStyle = 'red'

    let n = 0
    this.interval = setInterval(()=>{
      n++
      let h = parseInt(n / 3600)
      let m = parseInt(n / 60)
      m = m - (h * 60)
      let s = parseInt(n % 60)
      let timeStr = this.toDub(h) + ":"+ this.toDub(m) + ":" + this.toDub(s)
      this.setTimeStr(timeStr)
    }, 1000)
  }
  setTimeStr = (timeStr) => {
    this.clearTimeStr()
    this.context.fillText(timeStr, this.canvas.width/2, this.canvas.height/2+5)
  }

  clearTimeStr = () => {
    if (this.context) {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height)
    }
  }

  stopClock = () => {
    clearInterval(this.interval)
  }
  toDub = (n) => {
    return n < 10 ? "0" + n : "" + n
  }
  close = async () => {
    const {
        overlayStatus,
        changePaddingStatus,
        stopRecording,
        getRecording,
        videoSetting,
        stopPlay
    } = this.props
    try {
      changePaddingStatus(true)
      const {resolution, resolutionSelected } = videoSetting
      let type = resolution[resolutionSelected]
      if(type == "4K") await new Promise(r => setTimeout(() => r(stopPlay(false)), 200))
      await stopRecording()
      await getRecording()

      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
      if (this.props.recordingStatus) {
        layer.msg('Close videotape Fail！Please try again later.', { icon: 5 })
      } else {
        this.stopClock()
        $("#record_time").remove()
        layer.msg('Close videotape Success!', { icon: 6 })
      }
    } catch (error) {
      !error.deal && console.log(error)
      setTimeout(() => {
        changePaddingStatus(false)
      }, 200)
      layer.msg('Close videotape Fail！Please try again later.', { icon: 5 })
    }
  }

  render () {
    const { recordingStatus } = this.props
    let className = classNames({
      open: recordingStatus
    })
    return (
        <a id="recording" className={className} onClick={this.onClick}></a>
    )
  }
}

const mapStateToProps = state => ({
  liveStatus: state.live.liveStatus,
  overlayStatus: state.overlay.switchStatus,
  recordingStatus: state.recording.recordingStatus,
  padding: state.padding,
  videoSetting: state.video.setting,
  playStatus: state.recording.playStatus
})

const mapDispatchToProps = {
  startRecording,
  getRecording,
  stopRecording,
  closeOverlay,
  changePaddingStatus,
  stopPlay
}

export default connect(mapStateToProps, mapDispatchToProps)(RecordBtn)
