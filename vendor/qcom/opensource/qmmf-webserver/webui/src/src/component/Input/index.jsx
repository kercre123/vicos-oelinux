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
import styles from './Input.sass'
import classNames from 'classnames/bind'
const cx = classNames.bind(styles)

class Input extends Component {
  onChange = e => {
    if (this.props.type === 'number' && e.target.value < 0) {
      return
    }
    const evt = {
      baseEvent: e,
      name: this.props.name,
      value: e.target.value
    }
    this.props.onChange(evt)
  }

  renderError = () => {
    if (this.props.error && this.props.showError) {
      return (
        <label className="error">{this.props.errorMsg}</label>
      )
    }
  }

  render () {
    const { labelName, value } = this.props
    const className = cx('Input', this.props.className)
    let inputProps = {
      disabled: this.props.disabled,
      placeholder: this.props.placeholder,
      value: this.props.value
    }
    if (this.props.type) {
      inputProps.type = this.props.type
    }
    let labelProps = {}
    if (this.props.labelClass) {
      labelProps.className = this.props.labelClass
    }
    return (
      <div className={className}>
        <label {...labelProps}>{labelName}</label>
        <input value={value} onChange={this.onChange} {...inputProps}/>
        {this.renderError()}
      </div>
    )
  }
}

Input.propTypes = {
  labelName: PropTypes.string.isRequired,
  onChange: PropTypes.func.isRequired
}

export default Input
