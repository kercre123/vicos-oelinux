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
import classNames from 'classnames'
import './FaceRegisterBtn.sass'
import { switchFaceRegisterLayer, cutImage } from 'src/store/faceRegister'
import { sendPostTakingPictures } from 'src/ajax'

class FaceRegisterBtn extends Component {
  onClick = () => {
    const { faceRecognitionLiveStatus, channelSelect } = this.props
    if (!faceRecognitionLiveStatus) {
      layer.msg(
        'Please first open the face recognition function',
        { icon: 5, area:380 }
      )
      return
    }
    let { width, height, x, y } = this.props.canvas.getRegisterFacePosition()
    sendPostTakingPictures().then(data => {
      if (data.Data) {
        let img = `data:image/jpeg;base64,${data.Data}`
        let imgEl = document.createElement('img')
        imgEl.src = img
        let canvas = document.createElement('canvas')
        canvas.height = height
        canvas.width = width
        let ctx = canvas.getContext('2d')
        imgEl.onload = () => {
          let ret = { width, height }
          ctx.drawImage(imgEl, x, y, width, height, 0, 0, width, height)
          let idata = ctx.getImageData(0, 0, width, height)
          ret.data = canvas.toDataURL('image/jpeg', 1)
          let rgba = idata.data
          let gray8 = new Uint8Array(width * height)
          let row_stride = rgba.length / height
          let padding = row_stride - 4 * width
          let idx_gray8 = 0
          let idx_rgba = 0
          for (y = 0; y < height; y++) {
              for (x = 0; x < width; x++) {
                  let luma = rgba[idx_rgba] * 0.2989 + rgba[idx_rgba + 1] * 0.5870 + rgba[idx_rgba + 2] * 0.1140
                  gray8[idx_gray8++] = luma
                  idx_rgba += 4
              }
              idx_rgba += padding
          }
          var len = gray8.length;
          let binary = ''
          for (var i = 0; i < len; i++) {
              binary += String.fromCharCode( gray8[ i ] );
          }
          ret.lumaData = btoa(binary)
          this.props.switchFaceRegisterLayer(true)
          this.props.cutImage(ret)
        }
      }
    }).catch(error => error.deal || console.log(error))
  }

  render () {

    const { faceRecognitionLiveStatus } = this.props

    let className = classNames({
      open: faceRecognitionLiveStatus
    })

    return (
      <a id='face-register' className={className} onClick={this.onClick}></a>
    )
  }
}

const mapStateToProps = state => ({
  channelSelect: state.channel.selected,
  canvas: state.live.canvas,
  faceRegister: state.faceRegister,
  faceRecognitionLiveStatus: state.faceRecognition.faceRecognitionLiveStatus
})

const mapDispatchToProps = {
  switchFaceRegisterLayer,
  cutImage
}

export default connect(mapStateToProps, mapDispatchToProps)(FaceRegisterBtn)
