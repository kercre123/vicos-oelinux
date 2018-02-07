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
import classNames from 'classnames'
import { openOverlay, closeOverlay } from 'store/overlay'
import { changePaddingStatus } from 'store/padding'
import './overlayBtn.sass'

class OverlayBtn extends Component {
  onClick = () => {
    const { padding, overlayStatus } = this.props
    if (padding) {
      return
    }
    if (overlayStatus) {
      return this.close()
    }
    return this.open()
  }

  open = async () => {
    console.log('open')
    const { liveStatus, recordingStatus, openOverlay } = this.props
    if (!recordingStatus && !liveStatus) {
      return layer.msg('Please open preview or rec first', { icon: 5 })
    }
    try {
      await changePaddingStatus(true)
      await openOverlay()
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
    const { changePaddingStatus, closeOverlay } = this.props
    try {
      await changePaddingStatus(true)
      await closeOverlay()
      await changePaddingStatus(false)
    } catch (error) {
      !error.deal && console.log(error)
      changePaddingStatus(false)
    }
  }

  render () {
    const { overlayStatus } = this.props
    let className = classNames({
      open: overlayStatus
    })
    return (
      <a id="overlay" className={className} onClick={this.onClick}></a>
    )
  }
}

const mapStateToProps = state => ({
  overlayStatus: state.overlay.switchStatus,
  recordingStatus: state.recording.recordingStatus,
  liveStatus: state.live.liveStatus,
  padding: state.padding,
})

const mapDispatchToProps = {
  openOverlay,
  closeOverlay,
  changePaddingStatus
}

export default connect(mapStateToProps, mapDispatchToProps)(OverlayBtn)
