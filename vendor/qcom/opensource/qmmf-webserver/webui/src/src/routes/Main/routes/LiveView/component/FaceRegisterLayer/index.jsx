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
import './FaceRegisterLayer.sass'
import { switchFaceRegisterLayer } from 'store/faceRegister'
import { sendPostFaceRegister } from 'ajax'

class FaceRegisterLayer extends Component {
  onCancelClick = () => {
    this.props.switchFaceRegisterLayer(false)
  }

  onPrimaryClick = () => {
    const { lumaData, width, height } = this.props.image
    let base64Data = lumaData
    base64Data = base64Data.replace(/\+/g, '%2B')
    base64Data = base64Data.replace(/\//g, '%2F')
    const OBJECT_TYPE_FACE = 3
    const EVENT_TYPE_FR = 8
    const FORMAT_GRAY8 = 7
    const name = this.refs.name.value.trim()
    let msg, icon
    if (!name.match(/^[a-zA-Z0-9_]{1,20}$/)) {
      msg = 'Please enter 1-20 letters, numbers and underscores.'
      icon = 5
      layer.msg(msg, { icon })
    }
   else{
      let reqData = "vam_enroll_object_type=" + OBJECT_TYPE_FACE +
          "&vam_enroll_event_type=" + EVENT_TYPE_FR +
          "&vam_enroll_format=" + FORMAT_GRAY8 +
          "&vam_enroll_width=" + width +
          "&vam_enroll_height=" + height +
          "&vam_enroll_id=" + name +
          "&vam_enroll_name=" + name +
          "&vam_enroll_data=" + base64Data
      this.props.switchFaceRegisterLayer(false)
      return sendPostFaceRegister(reqData).then(data => {
        this.props.switchFaceRegisterLayer(false)
        this.props.canvas.clearRegisterId()
        if (data.status) {
          msg = 'Register success！'
          icon = 6
        } else {
          msg = 'Register Fail！Please try again later.'
          icon = 5
        }
        layer.msg(msg, { icon })
      }).catch(error => {
        error.deal || console.log(error)
        this.props.canvas.clearRegisterId()
        this.props.switchFaceRegisterLayer(false)
      })
    }
  }

  render () {
    const { layerStatus, image } = this.props
    const innerStyle = {
      display: layerStatus ? '' : 'none'
    }
    if (this.refs.image && image !== null) {
      this.refs.image.src = image.data
      this.refs.image.height = 300 / image.width * image.height
    }

    return (
      <div id="face-register-layer" style={innerStyle}>
        <div className="register-box">
          <div className="register-img-area">
            <img width="300" ref="image"></img>
          </div>
          <div className="register-input-area">
              <input ref="name" className="register-input" placeholder="Name"></input>
          </div>
          <div className="register-button-area">
            <button className="secondary" onClick={this.onCancelClick}>Cancel</button>
            <button className="primary" onClick={this.onPrimaryClick}>OK</button>
          </div>
        </div>
      </div>
    )
  }
}

const mapStateToProps = state =>({
  canvas: state.live.canvas,
  layerStatus: state.faceRegister.layerStatus,
  image: state.faceRegister.image
})

const mapDispatchToProps = {
  switchFaceRegisterLayer
}

export default connect(mapStateToProps, mapDispatchToProps)(FaceRegisterLayer)
