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
import Layout from '../component/Layout'
import Input from 'src/component/Input'
import Select from 'src/component/Select'
import { sendGetWifi, sendPostWifi } from 'src/ajax'
import layer from 'layerUI'

class wifiAddressContainer extends Component {
  constructor (props) {
    super(props)
    this.state = {
      modeSelectVal: 0,
      mode: [],
      ssid: '',
      password: '',
      ssidErr: false,
      passwordErr: false
    }
  }

  onChange = e => {
    this.setState({
      [e.name]: e.value
    })
    if (e.name === 'password') {
      this.setState({
        passwordErr: false
      })
    }
  }

  onModeSelected = e => {
    if (this.state.modeSelectVal !== e.value) {
      this.setState({
        modeSelectVal: e.value
      })
    }
  }

  setNetwork = () => {
    const { modeSelectVal, ssid, password } = this.state
    if (ssid.length < 8 || ssid.length > 32) {
      this.setState({
        ssidErr: true
      })
      return
    }
    if (password.length < 8 || password.length > 16) {
      this.setState({
        passwordErr: true
      })
      return
    }
    let reqData = {}
    reqData.modeSelectVal = modeSelectVal
    reqData.ssid = ssid
    reqData.password = password
    return sendPostWifi(reqData).then(data => {
      const { status } = data
      let msg, icon
      if (status) {
        msg = 'Set Wifi Setting success, please re login!'
        icon = 6
        layer.msg(msg, { icon }, () => {
          window.sessionStorage.removeItem('SESSION_USER')
          if (window.navigator.userAgent.indexOf("Firefox") !== -1 || window.navigator.userAgent.indexOf("Chrome") !== -1) {

            window.location.href="about:blank";
          } else {
             window.opener = null;
             window.open("", "_self");
             window.close();
          }
        })
      } else {
        msg = 'Set Wifi Setting Fail！Please try again later.'
        icon = 5
        layer.msg(msg, { icon })
      }
    }).catch(error => {
      let msg, icon
      if (error.message == 'Request Timeout') {
        msg = 'Set Wifi Setting success, please re login!'
        icon = 6
        layer.msg(msg, { icon }, () => {
          window.sessionStorage.removeItem('SESSION_USER')
          if (window.navigator.userAgent.indexOf("Firefox") !== -1 || window.navigator.userAgent.indexOf("Chrome") !== -1) {

            window.location.href="about:blank";
          } else {
             window.opener = null;
             window.open("", "_self");
             window.close();
          }
        })
      } else {
        msg = 'Set Wifi Setting Fail！Please try again later.'
        icon = 5
        layer.msg(msg, { icon })
      }
    })
  }

  componentWillMount () {
    sendGetWifi().then(data => {
      if (data.status) {
        this.setState({...data})
      }
    })
  }

  render () {
    const { modeSelectVal, ssid, password, mode, passwordErr, ssidErr } = this.state

    const inputProps = {
      onChange: this.onChange
    }

    return (
      <Layout>
        <div id="wifi_address_form" className="setup-form">
          <div className="header">WIFI</div>
          <div className="main">
            <Select labelName="Wifi Mode:"
              options={mode}
              selected={parseInt(modeSelectVal)}
              selectName="modeSelectVal"
              onChange={this.onChange}>
            </Select>
            <Input labelName="SSID:" name="ssid" value={ssid}
              placeholder="ssid" {...inputProps}
              error={true} showError={ssidErr}
              errorMsg="SSID length must be between 8 to 32"/>
            <Input labelName="Password:" name="password" value={password}
              placeholder="password" {...inputProps}
              error={true} showError={passwordErr}
              errorMsg="Password length must be between 8 to 16"/>
          </div>
          <div className="footer">
            <button className="primary"
              onClick={this.setNetwork}>
                Save
            </button>
          </div>
        </div>
      </Layout>
    )
  }
}

export default wifiAddressContainer
