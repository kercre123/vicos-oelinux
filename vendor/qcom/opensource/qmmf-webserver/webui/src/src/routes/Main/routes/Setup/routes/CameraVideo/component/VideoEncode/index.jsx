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
import Select from 'component/Select'
import { getVideoSetting, setVideoSetting } from 'store/videoSetting'

class VideoEncode extends Component {
  constructor (props) {
    super(props)
    this.state = {
      setting: {
        resolution: [],
        resolutionSelected: 0,
        encodeMode: [],
        encodeModeSelected: 0,
        bitRate: [],
        bitRateSelected: 0,
        fps: [],
        fpsSelected: 0
      }
    }
  }

  componentWillMount () {
    this.props.getVideoSetting().then(() => {
      this.setState({
        setting: this.props.setting
      })
    })
  }

  onChange = e => {
    const { name, value } = e
    const setting = Object.assign({}, this.state.setting, { [name]: value })
    this.setState({
      setting
    })
  }

  sendVideoSetting = () => {
    const { setting } = this.state
    this.props.setVideoSetting(setting).then(status => {
      let msg, icon
      if (status) {
        msg = 'Set Video Setting success！'
        icon = 6
      } else {
        msg = 'Set Video Setting Fail！Please try again later.'
        icon = 5
      }
      layer.msg(msg, { icon })
    }).catch(error => {
      layer.msg('Set Video Setting Fail！Please try again later.', { icon: 5 })
    })
  }

  render () {
    const {
      resolution, resolutionSelected,
      encodeMode, encodeModeSelected,
      bitRate, bitRateSelected,
      fps, fpsSelected
    } = this.state.setting

    return (
      <div id="video_encode_form" className="setup-form">
        <div className="header">Video Encode</div>
        <div className="main">
          <Select labelName="Encode Mode:"
            options={encodeMode}
            selected={parseInt(encodeModeSelected)}
            selectName="encodeModeSelected"
            onChange={this.onChange}>
          </Select>
          <Select labelName="Resolution:"
            options={resolution}
            selected={parseInt(resolutionSelected)}
            selectName="resolutionSelected"
            onChange={this.onChange}>
          </Select>
          <Select labelName="Bit Rate:"
            options={bitRate}
            selected={parseInt(bitRateSelected)}
            selectName="bitRateSelected"
            onChange={this.onChange}>
          </Select>
          <Select labelName="FPS For Video:"
            options={fps}
            selected={parseInt(fpsSelected)}
            selectName="fpsSelected"
            onChange={this.onChange}>
            fps
          </Select>
        </div>
        <div className="footer">
          <button className="primary"
            onClick={this.sendVideoSetting}>
              Save
          </button>
        </div>
      </div>
    )
  }
}

const mapStateToProps = state => ({
  setting: state.video.setting
})

const mapDispatchToProps = {
  getVideoSetting,
  setVideoSetting
}

export default connect(mapStateToProps, mapDispatchToProps)(VideoEncode)
