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
import { openVam, closeVam } from 'src/store/vam'
import { openFaceRecognition, closeFaceRecognition, changeFaceRecognitionLiveStatus } from 'src/store/faceRecognition'
import classNames from 'classnames'
import { changePaddingStatus } from 'store/padding'
import './faceRecognitionBtn.sass'
import Promise from 'bluebird'

class FaceRecognitionBtn extends Component {
  onClick = () => {
    const { padding, faceRecognitionLiveStatus, channelSelect } = this.props
    if (padding) {
      return
    }
    if (faceRecognitionLiveStatus) {
      return this.close()
    }
    return this.open()
  }

  open = async () => {
    const {
      liveStatus,
      changePaddingStatus,
      openVam,
      openFaceRecognition,
      changeFaceRecognitionLiveStatus,
    } = this.props
    if (!liveStatus) {
      return layer.msg('Please open preview first', { icon: 5 })
    }
    try {
      changePaddingStatus(true)
      await openVam('FR')
      await new Promise(r => setTimeout(() => r(openFaceRecognition()), 1000))
      await new Promise(r => setTimeout(() => r(changeFaceRecognitionLiveStatus(true)), 500))
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
      changeFaceRecognitionLiveStatus,
      closeFaceRecognition,
      closeVam,
    } = this.props
    try {
      changePaddingStatus(false)
      await changeFaceRecognitionLiveStatus(false)
      await new Promise(r => setTimeout(() => r(closeFaceRecognition()), 800))
      await new Promise(r => setTimeout(() => r(closeVam('FR')), 500))
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
    const { faceRecognitionLiveStatus } = this.props
    let className = classNames({
      open: faceRecognitionLiveStatus
    })

    return (
      <a id="face-recognition" className={className} onClick={this.onClick}></a>
    )
  }
}

const mapStateToProps = state => ({
  channelSelect: state.channel.selected,
  liveStatus: state.live.liveStatus,
  vam: state.vam,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus,
  padding: state.padding
})

const mapDispatchToProps = {
  openVam,
  closeVam,
  openFaceRecognition,
  closeFaceRecognition,
  changeFaceRecognitionLiveStatus,
  changePaddingStatus
}

export default connect(mapStateToProps, mapDispatchToProps)(FaceRecognitionBtn)
