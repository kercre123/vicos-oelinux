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
import { sendPostTakingPictures } from 'src/ajax'
import { changePaddingStatus } from 'store/padding'
import { connect } from 'react-redux'
import layer from 'layerUI'
import './takePicBtn.sass'

class TakePicBtn extends Component {
  takingPictures = () => {
    if (this.props.padding) {
      return
    }
    this.props.changePaddingStatus(true)
    return sendPostTakingPictures().then(data => {
      let msg
      let icon
      if (data.Data) {
        msg = 'Taking picture success!'
        icon = 6
      } else {
        msg = 'Taking picture Fail! Please try again later.'
        icon = 5
      }
      layer.msg(msg, { icon })
      this.props.changePaddingStatus(false)
    }).catch(error => {
      !error.deal && console.log(error)
      console.log('catch error')
      let msg = 'Taking picture Fail! Please try again later.'
      let icon = 5
      layer.msg(msg, { icon })
      this.props.changePaddingStatus(false)
    })
  }

  render () {
    return (
      <a id="take-pic" onClick={this.takingPictures}></a>
    )
  }
}

const mapStateToProps = state => ({
  padding: state.padding,
})

const mapDispatchToProps = {
  changePaddingStatus,
}

export default connect(mapStateToProps, mapDispatchToProps)(TakePicBtn)
