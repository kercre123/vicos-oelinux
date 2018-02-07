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
import SlideBtn from 'src/component/SlideBtn'
import { openVam, closeVam } from 'src/store/vam'
import { changePaddingStatus } from 'store/padding'
import { openCameraTamper, closeCameraTamper, changeCameraTamperLiveStatus } from 'src/store/cameraTamper'
import Promise from 'bluebird'

class CameraTamper extends Component {
  onClick = () => {
    const { padding, cameraTamperLiveStatus, channelSelect } = this.props
    if (padding) {
      return
    }
    if (cameraTamperLiveStatus) {
      return this.close()
    }
    return this.open()
  }

  open = async () => {
    const {
      liveStatus,
      changePaddingStatus,
      openVam,
      openCameraTamper,
      changeCameraTamperLiveStatus,
      motionDetectionLiveStatus
    } = this.props
    if (!motionDetectionLiveStatus) {
      return layer.msg('Please enable motion detection first', { icon: 5 })
    }
    if (!liveStatus) {
      return layer.msg('Please open preview first', { icon: 5 })
    }
    try {
      changePaddingStatus(true)
      await openVam('CT')
      await new Promise(r => setTimeout(() => r(openCameraTamper()), 500))
      await new Promise(r => setTimeout(() => r(changeCameraTamperLiveStatus(true)), 500))
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
      changePaddingStatus,
      changeCameraTamperLiveStatus,
      closeCameraTamper,
      closeVam,
    } = this.props
    try {
      changePaddingStatus(true)
      await new Promise(r => setTimeout(() => r(closeVam('CT')), 500))
      await new Promise(r => setTimeout(() => r(closeCameraTamper()), 800))
      await changeCameraTamperLiveStatus(false)
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
    const { cameraTamperLiveStatus, vamStatus } = this.props
    const props = {
      open: cameraTamperLiveStatus,
      name: 'Camera Tamper',
      onClick: this.onClick
    }

    return (
      <SlideBtn {...props}></SlideBtn>
    )
  }
}

const mapStateToProps = state => ({
  channelSelect: state.channel.selected,
  vamStatus: state.vam.vamStatus,
  liveStatus: state.live.liveStatus,
  cameraTamperLiveStatus: state.cameraTamper.cameraTamperLiveStatus,
  padding: state.padding,
  motionDetectionLiveStatus: state.motionDetection.motionDetectionLiveStatus
})

const mapDispatchToProps = {
  openCameraTamper,
  closeCameraTamper,
  changePaddingStatus,
  openVam,
  closeVam,
  changeCameraTamperLiveStatus,
}

export default connect(mapStateToProps, mapDispatchToProps)(CameraTamper)
