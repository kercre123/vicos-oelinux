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
import Input from 'component/Input'
import { getOverlayConfig, setOverlayConfig } from 'store/overlay'
import layer from 'layerUI'
import { splitObject } from 'src/util'

const overlayMap = {
  datetime: [
    { name: 'ov_position', type: 'select', label: 'Overlay Position:' },
    { name: 'ov_color', type: 'input', label: 'Overlay Color:', inputType: 'color' },
    { name: 'ov_date', type: 'select', label: 'Overlay Date Type:' },
    { name: 'ov_time', type: 'select', label: 'Overlay Time Type:' }
  ],
  usertext: [
    { name: 'ov_position', type: 'select', label: 'Overlay Position:' },
    { name: 'ov_color', type: 'input', label: 'Overlay Color:', inputType: 'color' },
    { name: 'ov_usertext', type: 'input', label: 'Overlay Content:', inputType: 'text' }
  ],
  boundingbox: [
    { name: 'ov_box_name', type: 'input', label: 'Overlay Box Name:', inputType: 'text' },
    { name: 'ov_color', type: 'input', label: 'Overlay Color:', inputType: 'color' },
    { name: 'ov_start_x', type: 'input', label: 'Overlay Start X:', inputType: 'number' },
    { name: 'ov_start_y', type: 'input', label: 'Overlay Start Y:', inputType: 'number' },
    { name: 'ov_width', type: 'input', label: 'Overlay Width:', inputType: 'number',
      errorMsg: 'Overlay Width must greater than zero', showError: false, error: true },
    { name: 'ov_height', type: 'input', label: 'Overlay Height:', inputType: 'number',
      errorMsg: 'Overlay Height must greater than zero', showError: false, error: true }
  ],
  staticimage: [
    { name: 'ov_position', type: 'select', label: 'Overlay Position:' },
    { name: 'ov_image_location', type: 'input', label: 'Overlay Image:', inputType: 'text' },
    { name: 'ov_width', type: 'input', label: 'Overlay Width:', inputType: 'number',
      errorMsg: 'Overlay Width must greater than zero', showError: false, error: true },
    { name: 'ov_height', type: 'input', label: 'Overlay Height:', inputType: 'number',
      errorMsg: 'Overlay Height must greater than zero', showError: false, error: true }
  ],
  privacymask: [
    { name: 'ov_color', type: 'input', label: 'Overlay Color:', inputType: 'color' },
    { name: 'ov_start_x', type: 'input', label: 'Overlay Start X:', inputType: 'number' },
    { name: 'ov_start_y', type: 'input', label: 'Overlay Start Y:', inputType: 'number' },
    { name: 'ov_width', type: 'input', label: 'Overlay Width:', inputType: 'number',
      errorMsg: 'Overlay Width must greater than zero', showError: false, error: true },
    { name: 'ov_height', type: 'input', label: 'Overlay Height:', inputType: 'number',
      errorMsg: 'Overlay Height must greater than zero', showError: false, error: true }
  ]
}

class Overlay extends Component {
  constructor (props) {
    super(props)
    this.state = {
      overlay: this.props.overlay,
      color: '#FFFFFF',
      alpha: 0
    }
  }

  changeColor = e => {
    const overlay = Object.assign({}, this.state.overlay, { ov_color: this.rgbToDecimal(e.value) })
    this.setState({ color: e.value, overlay })
  }

  onChange = e => {
    const { name, value } = e
    const type = e.baseEvent.target.type
    const overlay = Object.assign({}, this.state.overlay, {
      [name]: type === 'number' ? parseInt(value, 10) : value
    })
    this.setState({
      overlay
    })
  }

  onTypeChange = e => {
    const { name, value } = e
    this.props.getOverlayConfig({ [name]: value }).then(() => {
      this.setState({
        overlay: this.props.overlay,
        color: this.decimalToRgb(this.props.overlay.ov_color)
      })
    })
  }

  setOverlay = () => {
    const { overlay } = this.state
    const { ov_type, ov_type_SelectVal } = overlay
    let args = {
      ov_type_SelectVal
    }
    let validateResults = true
    overlayMap[ov_type[ov_type_SelectVal]].every(item => {
      const { name, type } = item
      if (type === 'input') {
        args[name] = overlay[name]
      } else if (type === 'select') {
        args[name + '_SelectVal'] = overlay[name + '_SelectVal']
      }

      if (name === 'ov_usertext' && overlay[name].trim() === '') {
        layer.msg('Overlay Content can\'t be empty, Please input.', { icon: 5 })
        validateResults = false
        return false
      }

      if (name === 'ov_box_name' &&  !overlay[name].match(/^[a-zA-Z0-9_\s]{1,22}$/)) {
        layer.msg('Please enter 1-22 letters,numbers,underscores and spaces.', { icon: 5, area:450 })
        validateResults = false
        return false
      }

      if (name === 'ov_usertext' && !overlay[name].match(/^[a-zA-Z0-9_\s]{1,25}$/)) {
        layer.msg('Please enter 1-25 letters,numbers,underscores and spaces.', { icon: 5, area:450 })
        validateResults = false
        return false
      }

      if (name === 'ov_start_x' && (overlay[name] > 1920 || overlay[name] < 0)) {
        layer.msg('Overlay start X must between 0 and 1920', { icon: 5 })
        validateResults = false
        return false
      }

      if (name === 'ov_start_y' && (overlay[name] > 1080 || overlay[name] < 0)) {
        layer.msg('Overlay start Y must between 0 and 1080', { icon: 5 })
        validateResults = false
        return false
      }

      if ((name === 'ov_width' || name === 'ov_height') && overlay[name] < 1) {
        layer.msg('Overlay Width & Height must greater than 0', { icon: 5 })
        validateResults = false
        return false
      }

      return true
    })

    validateResults && this.props.setOverlayConfig(args).then(status => {
      let msg
      let icon
      if (status) {
        msg = 'Set Overlay Config success！'
        icon = 6
        layer.msg(msg, { icon })
      } else {
        msg = 'Set Overlay Config Fail！Please try again later.'
        icon = 5
        layer.msg(msg, { icon, area: '380px' })
      }
    }).catch(error => {
      layer.msg('Set Overlay Config Fail！Please try again later.', { icon: 5 })
    })
  }

  rgbToDecimal = value => {
    let a = this.state.alpha
    let red = parseInt(value.slice(1, 3), 16)
    let green = parseInt(value.slice(3, 5), 16)
    let blue = parseInt(value.slice(5, 7), 16)
    console.log(`rgb: ${value}, to dec: ${(red << 24 | green << 16 | blue << 8 | a) >>> 0}`)
    return ((red << 24 | green << 16 | blue << 8 | a) >>> 0).toString()
  }

  decimalToRgb = val => {
    let dec = parseInt(val, 10) >>> 0
    let red = (dec >> 24) & 0xFF
    let green = (dec >> 16) & 0xFF
    let blue = (dec >> 8) & 0xFF
    let alpha = dec & 0xFF
    this.setState({
      alpha
    })
    let str = '#'
    str += red > 16 ? red.toString(16).toUpperCase() : '0' + red.toString(16).toUpperCase()
    str += green > 16 ? green.toString(16).toUpperCase() : '0' + green.toString(16).toUpperCase()
    str += blue > 16 ? blue.toString(16).toUpperCase() : '0' + blue.toString(16).toUpperCase()

    console.log(`dec: ${val}, to rgb: ${str}`)
    return str
  }

  componentDidMount () {
    this.props.getOverlayConfig().then(() => {
      this.setState({
        overlay: this.props.overlay,
        color: this.decimalToRgb(this.props.overlay.ov_color)
      })
    })
  }

  makeItem = (props) => {
    const [{ name, type, label, inputType }, others] = splitObject(props,
      ['name', 'type', 'label', 'inputType'])
    const { overlay } = this.state
    if (type === 'select') {
      return (
        <Select key={name} labelName={label} options={overlay[name]}
          selected={parseInt(overlay[name + '_SelectVal'], 10)}
          selectName={name + '_SelectVal'} onChange={this.onChange} {...others}>
        </Select>
      )
    } else if (type === 'input') {
      if (inputType === 'color') {
        return (
          <Input key={name} labelName={label} labelClass="color" name="color"
            onChange={this.changeColor} value={this.state.color} type="color" {...others}/>
        )
      }
      return (
        <Input key={name} labelName={label} name={name}
          onChange={this.onChange} value={overlay[name]} type={inputType}  {...others}/>
      )
    }
  }

  renderItems = type => {
    return overlayMap[type].map(item => this.makeItem(item))
  }

  render () {
    const { ov_type, ov_type_SelectVal, isActive } = this.state.overlay

    return (
      <div id="overlay_form" className="setup-form">
        <div className="header">Overlay</div>
        <div className="main">
          <Select labelName="Overlay Type:"
            options={ov_type}
            selected={parseInt(ov_type_SelectVal, 10)}
            selectName="ov_type_SelectVal"
            onChange={this.onTypeChange}>
          </Select>
          {ov_type.length ? this.renderItems(ov_type[ov_type_SelectVal]) : ''}
        </div>
        <div className="footer">
          <button className="primary"
            onClick={this.setOverlay}>
              Save
          </button>
        </div>
      </div>
    )
  }
}

const mapStateToProps = state => ({
  overlay: state.overlay
})

const mapDispatchToProps = {
  getOverlayConfig,
  setOverlayConfig
}

export default connect(mapStateToProps, mapDispatchToProps)(Overlay)
