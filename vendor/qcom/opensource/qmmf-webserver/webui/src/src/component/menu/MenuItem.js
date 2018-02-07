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

import * as React from 'react'
import classNames from 'classnames'

class MenuItem extends React.Component {
  static propTypes = {
    eventKey: React.PropTypes.string,
    selectedKeys: React.PropTypes.array,
    onSelect: React.PropTypes.func,
    onDeselect: React.PropTypes.func
  }

  constructor (props) {
    super(props)
  }

  onClick = (e) => {
    const selected = this.isSelected()
    const props = this.props
    const { eventKey } = props
    const info = {
      key: eventKey,
      keyPath: [eventKey],
      item: this,
      domEvent: e
    }
    if (!selected) {
      props.onSelect(info)
    }
  }

  isSelected = () => {
    return this.props.selectedKeys.indexOf(this.props.eventKey) !== -1
  }

  render () {
    const { children } = this.props

    let mouseEvent = {
      onClick: this.onClick
    }

    let className = classNames({
      'menu-item': true,
      'is-selected': this.isSelected()
    })

    return (<li className={className} {...mouseEvent}>{children}</li>)
  }
}

export default MenuItem
