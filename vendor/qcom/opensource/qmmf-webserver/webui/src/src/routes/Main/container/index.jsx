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

import React, { Component } from 'react'
import { connect } from 'react-redux'
import Header from 'component/Header'
import { sendLogout } from 'ajax'
import { hashHistory } from 'react-router'
import { closeLive } from 'store/live'
import { setPreview } from 'store/preview'
import { stopRecording } from 'store/recording'
import { clearSocket } from 'store/socket'
import { closeOverlay } from 'store/overlay'
import { uri } from 'src/settings'
import { getCookie } from 'src/util'
import { closeVam } from 'src/store/vam'
import { closeFaceRecognition, changeFaceRecognitionLiveStatus } from 'src/store/faceRecognition'
import { closeMotionDetection, changeMotionDetectionLiveStatus } from 'src/store/motionDetection'
import { closeCameraTamper, changeCameraTamperLiveStatus } from 'src/store/cameraTamper'
import { initSocket, sendFirstKeepAlive, sendIntervalKeepAlive } from 'src/store/socket'
import { changePaddingStatus } from 'store/padding'

class mainContainer extends Component {
  componentDidMount () {
    let user = window.sessionStorage.getItem('SESSION_USER')
    if (user !== null && this.props.socket.connect === null) {
      let socket
      if (__DEV__) {
        socket = new WebSocket('ws://localhost:8003')
      } else {
        socket = new WebSocket(uri.ws)
      }

      this.props.initSocket(socket).then(status => {
        if (status === 'open') {
          this.props.sendFirstKeepAlive({ type : 'keep-alive', cookie: getCookie() })
          this.props.sendIntervalKeepAlive()
        }
      }).catch(error => {
        console.log(error)
      })
    }
  }

  componentWillReceiveProps (nextProps) {
    if (nextProps.location.pathname === '/') {
      window.location.reload()
    }
  }


  logout = async () => {
    if (this.props.padding) {
      return
    }
    const {
      liveStatus, setPreview, closeLive,
      overlayStatus, closeOverlay,
      recordingStatus, stopRecording,
      faceRecognitionLiveStatus, closeFaceRecognition, changeFaceRecognitionLiveStatus,
      motionDetectionLiveStatus, closeMotionDetection, changeMotionDetectionLiveStatus,
      cameraTamperLiveStatus, closeCameraTamper, changeCameraTamperLiveStatus,
      closeVam, changePaddingStatus
    } = this.props
    try {
      changePaddingStatus(true)
      overlayStatus && await closeOverlay()
      if (faceRecognitionLiveStatus) {
        changeFaceRecognitionLiveStatus(false)
        await new Promise(r => setTimeout(() => r(closeFaceRecognition()), 800))
        await new Promise(r => setTimeout(() => r(closeVam('FR')), 500))
      }
      if (motionDetectionLiveStatus) {
        changeMotionDetectionLiveStatus(false)
        await new Promise(r => setTimeout(() => r(closeMotionDetection()), 800))
        await new Promise(r => setTimeout(() => r(closeVam('MD')), 500))
      }
      if (cameraTamperLiveStatus) {
        changeCameraTamperLiveStatus(false)
        await new Promise(r => setTimeout(() => r(closeCameraTamper()), 800))
        await new Promise(r => setTimeout(() => r(closeVam('CT')), 500))
      }
      recordingStatus && await stopRecording()
      liveStatus && await closeLive().then(() => setPreview(false))
      await this.sendLogout()
      changePaddingStatus(false)
    } catch (error) {
      !error.deal && console.log(error)
      changePaddingStatus(false)
    }
  }

  sendLogout = () => {
    return sendLogout().then(data => {
      if (data.status) {
        window.sessionStorage.removeItem('SESSION_USER')
        this.props.clearSocket()
        hashHistory.push({ pathname: '/login' })
      }
    }).catch(error => console.log(error))
  }

  render () {
    return (
      <div className="body">
        <Header logout={this.logout} />
        <div id="main-area">
          {this.props.children}
        </div>
        <div id="footer_wrap">
            <label>Qualcomm Connected Camera</label>
        </div>
      </div>
    )
  }
}

const mapStateToProps = state => ({
  liveStatus: state.live.liveStatus,
  recordingStatus: state.recording.recordingStatus,
  overlayStatus: state.overlay.switchStatus,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus,
  motionDetectionLiveStatus: state.motionDetection.motionDetectionLiveStatus,
  cameraTamperLiveStatus: state.cameraTamper.cameraTamperLiveStatus,
  socket: state.socket,
  padding: state.padding
})

const mapDispatchToProps = {
  closeLive,
  stopRecording,
  clearSocket,
  closeVam,
  setPreview,
  closeFaceRecognition,
  closeMotionDetection,
  changeMotionDetectionLiveStatus,
  closeCameraTamper,
  changeCameraTamperLiveStatus,
  closeOverlay,
  initSocket,
  sendFirstKeepAlive,
  sendIntervalKeepAlive,
  changePaddingStatus,
  changeFaceRecognitionLiveStatus
}

export default connect(mapStateToProps, mapDispatchToProps)(mainContainer)
