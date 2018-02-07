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
import Radio from 'src/component/Radio'
import { sendGetNetwork, sendPostNetwork } from 'src/ajax'
import layer from 'layerUI'

class LanAddressContainer extends Component {
  constructor (props) {
    super(props)
    this.state = {
      modeSelectVal: 0,
      mode: [],
      ip: '',
      mask: '',
      range_start: '',
      range_end: '',
      ipError: false,
      maskError: false,
      range_startError: false,
      range_endError: false
    }
  }

  onChange = e => {
    this.setState({
      [e.name]: e.value,
      [`${e.name}Error`]: false,
    })
  }

  onModeSelected = e => {
    if (this.state.modeSelectVal === e.value) {
      return
    } else {
      this.setState({
        modeSelectVal: e.value,
        ipError: false,
        maskError: false,
        range_startError: false,
        range_endError: false
      })
    }
  }

  setNetwork = () => {
    let reqData = {}
    const { modeSelectVal, ip, mask, range_start, range_end } = this.state
    reqData.modeSelectVal = modeSelectVal
    if (this.state.modeSelectVal === 1) {
      var reg = /^(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])\.(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])\.(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])\.(\d{1,2}|1\d\d|2[0-4]\d|25[0-5])$/
      if (!reg.test(ip)) {
        this.setState({
          ipError: true
        })
        return
      }
      if (!reg.test(mask)) {
        this.setState({
          maskError: true
        })
        return
      }
      if (!reg.test(range_start)) {
        this.setState({
          range_startError: true
        })
        return
      }
      if (!reg.test(range_end)) {
        this.setState({
          range_endError: true
        })
        return
      }
      reqData.ip = ip
      reqData.mask = mask
      reqData.range_start = range_start
      reqData.range_end = range_end
    }
    return sendPostNetwork(reqData).then(data => {
      const { status } = data
      let msg, icon
      if (status) {
        msg = 'Set Network Setting success and login again！'
        icon = 6
        layer.msg(msg, { icon }, () => {
          window.sessionStorage.removeItem('SESSION_USER')
          userAgent = window.navigator.userAgen
          if (userAgent.indexOf("Firefox") !== -1 || userAgent.indexOf("Chrome") !== -1) {
            window.location.href="about:blank";
          } else {
            window.opener = null;
            window.open("", "_self");
            window.close();
          }
        })
      } else {
        msg = 'Set Network Setting Fail！Please try again later.'
        icon = 5
        layer.msg(msg, { icon, area:400 })
      }

    }).catch(error => {
      let msg, icon
      if (error.message == 'Request Timeout') {
        if (reqData.modeSelectVal == 0) {
          msg = 'Please use phone device discovery and login!'
          icon = 6
        } else {
          msg = 'Please use new address login!'
          icon = 6
        }
        layer.msg(msg, { icon ,area:400 }, () => {
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
        msg = 'Set Network Setting Fail！Please try again later.'
        icon = 5
        layer.msg(msg, { icon, area:400})
      }
    })
  }

  componentWillMount () {
    sendGetNetwork().then(data => {
      if (data.status) {
        this.setState({...data})
      }
    })
  }

  render () {
    const { modeSelectVal, ip, mask, range_start, range_end, ipError, maskError,range_startError,range_endError } = this.state
    const inputClass = {
      'disabled': modeSelectVal !== 1
    }

    const inputProps = {
      disabled: modeSelectVal !== 1,
      onChange: this.onChange
    }

    return (
      <Layout>
        <div id="lan_address_form" className="setup-form">
          <div className="header">LAN</div>
          <div className="main">
            <Radio content="DHCP" eventKey={0} onChange={this.onModeSelected} selected={modeSelectVal}/>
            <Radio content="Static IP Address" eventKey={1} onChange={this.onModeSelected} selected={modeSelectVal}/>
            <Input labelName="Address:" name="ip" {...inputProps} value={ip}
              placeholder="192.168.1.111" error={true} showError={ipError}
              errorMsg="IP Address format is incorrect"/>
            <Input labelName="Subnet:" name="mask"  {...inputProps} value={mask}
              placeholder="255.255.255.0" error={true} showError={maskError}
              errorMsg="Subnet format is incorrect"/>
            {
              modeSelectVal ? <Input labelName="range start:" name="range_start"  {...inputProps} value={range_start}
                placeholder="192.168.1.2" error={true} showError={range_startError}
              errorMsg="range start format is incorrect"/> :""
            }
            {
              modeSelectVal ? <Input labelName="range end:" name="range_end"  {...inputProps} value={range_end}
              placeholder="192.168.1.254" error={true} showError={range_endError}
              errorMsg="range end  format is incorrect"/> : ''
              }
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

export default LanAddressContainer
