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
import styles from './navList.sass'
import Item from './Item'

class NavList extends Component {
  constructor (props) {
    super(props)
    this.state = {
      menu: [
        {
          key: 'video',
          name: 'Video List'
        },
        {
          key: 'image',
          name: 'Image List'
        }
      ]
    }
  }

  onClick = e => {
    const { target } = e
    if (~this.props.selected.indexOf(target)) {
      return
    } else {
      this.props.onChange({ target: [target]})
    }
  }

  renderItem () {
    const { menu } = this.state
    const { selected } = this.props
    return menu.map(item => {
      return (
        <Item key={item.key} eventKey={item.key} selected={selected} onClick={this.onClick}>
          {item.name}
        </Item>
      )
    })
  }

  render () {
    return (
      <div className="navList">
        {this.renderItem()}
      </div>
    )
  }
}

NavList.propTypes = {
  selected: PropTypes.arrayOf(PropTypes.string.isRequired)
}

export default NavList
