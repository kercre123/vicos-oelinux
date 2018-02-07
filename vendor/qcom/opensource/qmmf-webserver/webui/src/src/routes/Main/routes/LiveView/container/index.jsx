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
import { debug } from 'util'
import Layout from '../component/Layout'
import ControlBar from '../component/ControlBar'
import LeftBar from '../component/LeftBar'
import ChannelSelect from '../component/ChannelSelect'
import VideoSetting from '../component/VideoSetting'
import TNR from '../component/TNR'
import SHDR from '../component/SHDR'
import MotionDetection from '../component/MotionDetection'
import CameraTamper from '../component/CameraTamper'
import RightBar from '../component/RightBar'
import LiveViewBtn from '../component/LiveViewBtn'
import FaceRecognitionBtn from '../component/FaceRecognitionBtn'
import FaceRegisterBtn from '../component/FaceRegisterBtn'
import FaceRegisterLayer from '../component/FaceRegisterLayer'
import OverlayBtn from '../component/OverlayBtn'
import TakePicBtn from '../component/TakePicBtn'
import RecordBtn from '../component/RecordBtn'
import LiveViewArea from '../component/LiveViewArea'
import { getChannel } from 'store/channel'
import { getVideoSetting } from 'store/videoSetting'
import { getParams } from 'store/params'
import { getPreview, setPreview } from 'store/preview'
import { getVam } from 'store/vam'
import { openLive, closeLive } from 'store/live'
import { getOverlay, closeOverlay } from 'store/overlay'
import { getRecording, stopRecording } from 'store/recording'
import { closeVam } from 'src/store/vam'
import { closeFaceRecognition, changeFaceRecognitionLiveStatus } from 'src/store/faceRecognition'
import { closeMotionDetection, changeMotionDetectionLiveStatus } from 'src/store/motionDetection'
import { closeCameraTamper, changeCameraTamperLiveStatus } from 'src/store/cameraTamper'
import { changePaddingStatus } from 'store/padding'
import Onvif from '../component/Onvif'
import { getOnvif } from 'store/onvif'
import AudioCheckbox from '../component/AudioCheckbox'

class LiveViewContainer extends Component {
  getInfo = async () => {
    const {
      getChannel, getPreview, getVam, getOverlay, getVideoSetting, getParams, getRecording,
      openLive, closeLive, getOnvif,
      changeFaceRecognitionLiveStatus, changeMotionDetectionLiveStatus, changeCameraTamperLiveStatus
    } = this.props
    try {
      changePaddingStatus(true)
      await getChannel()
      await getPreview()
      await getVam()
      await getOverlay()
      await getVideoSetting()
      await getParams()
      await getRecording()
      await getOnvif()
      await this.ListenerOnvif()
      this.props.preview.status ? await openLive() : await closeLive()
      if (this.props.vam.vamStatus) {
        const { mdUrl, frUrl, ctUrl } = this.props.vam
        frUrl && await changeFaceRecognitionLiveStatus(true)
        mdUrl && await changeMotionDetectionLiveStatus(true)
        ctUrl && await changeCameraTamperLiveStatus(true)
      }
      changePaddingStatus(false)
    } catch (error) {
      console.log(error)
      changePaddingStatus(false)
    }
  }

  componentDidMount () {
    this.getInfo()
    this.addEventListener()
  }

  addEventListener = () => {
    document.getElementById("root").addEventListener('click', this.wrapClick.bind(this), false)
  }

  wrapClick (evt) {
    const { onvif } = this.props
    if(evt.path[1].innerText !== "Onvif" && onvif.switchStatus === true) {
      if(onvif.promptState === true){
        layer.msg('Please turn off onvif, then reboot device to use.', {icon: 6, area: '380px' })
      }
      else{
        layer.msg('Please reboot device to use.', {icon: 6})
      }
      evt.stopPropagation()
    }
  }

  ListenerOnvif = () => {
    const { onvif } = this.props
    if(onvif.switchStatus === true){
      $("select").attr("disabled","disabled")
    }
    else{
      $("select").removeAttr("disabled")
    }
  }

  willUnmount = async () => {
    const {
      live: { liveStatus }, setPreview, closeLive,
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
      changePaddingStatus(false)
    } catch (error) {
      !error.deal && console.log(error)
      changePaddingStatus(false)
    }
  }

  componentWillUnmount () {
    this.willUnmount()
  }

  render () {
    const { videoSetting, live: { isChrome } } = this.props
    return (
      <Layout>
        <ControlBar>
          <LeftBar>
            <ChannelSelect changeEnd={this.getInfo}/>
            <VideoSetting setting={videoSetting}></VideoSetting>
            <Onvif />
            <TNR />
            <SHDR />
            { isChrome ? <MotionDetection /> : '' }
            { isChrome ? <CameraTamper />: ''}
            <AudioCheckbox/>
          </LeftBar>
          <RightBar>
            <LiveViewBtn/>
            { isChrome ? <FaceRecognitionBtn/> : ''}
            { isChrome ? <FaceRegisterBtn/> : ''}
            { isChrome ? <FaceRegisterLayer/> : ''}
            <OverlayBtn/>
            <RecordBtn/>
            <TakePicBtn/>
          </RightBar>
        </ControlBar>
        <LiveViewArea/>
      </Layout>
    )
  }
}

const mapStateToProps = state => ({
  channel: state.channel,
  videoSetting: state.video.setting,
  params: state.params,
  preview: state.preview,
  live: state.live,
  overlayStatus: state.overlay.switchStatus,
  recordingStatus: state.recording.recordingStatus,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus,
  motionDetectionLiveStatus: state.motionDetection.motionDetectionLiveStatus,
  cameraTamperLiveStatus: state.cameraTamper.cameraTamperLiveStatus,
  vam: state.vam,
  padding: state.padding,
  onvif: state.onvif
})

const mapDispatchToProps = {
  getChannel,
  getVideoSetting,
  getParams,
  getPreview,
  setPreview,
  getVam,
  openLive,
  closeLive,
  closeVam,
  closeFaceRecognition,
  changeFaceRecognitionLiveStatus,
  closeMotionDetection,
  changeMotionDetectionLiveStatus,
  closeCameraTamper,
  changeCameraTamperLiveStatus,
  getOverlay,
  closeOverlay,
  getRecording,
  stopRecording,
  changePaddingStatus,
  getOnvif
}

export default connect(mapStateToProps, mapDispatchToProps)(LiveViewContainer)
