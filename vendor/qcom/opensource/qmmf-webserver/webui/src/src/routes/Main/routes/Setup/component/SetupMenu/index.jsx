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
import SubMenu from './SubMenu'
import Item from './Item'
import './setupMenu.sass'

class SetupMenu extends Component {

  constructor (props) {
    super(props)
    const itemSelected = props.defaultSelected ? [props.defaultSelected] : []
    const openSelected = props.defaultOpenSelected ? props.defaultOpenSelected : []
    this.state = {
      openSelected,
      itemSelected
    }
  }

  onItemChange = e => {
    console.log(e)
    const itemSelected = this.state.itemSelected.concat()
    const key = e.eventKey
    const route = e.route
    if (itemSelected.length === 0) {
      this.setState({
        itemSelected: itemSelected.push(key)
      })
      return this.props.onItemClick(route)
    }
    const index = itemSelected.indexOf(key)
    if (index === -1) {
      this.setState({
        itemSelected: [key]
      })
      return this.props.onItemClick(e)
    }
  }

  onOpenChange = e => {
    const openSelected = this.state.openSelected.concat()
    const key = e.eventKey
    const index = openSelected.indexOf(key)
    if (index === -1) {
      openSelected.push(key)
    } else {
      openSelected.splice(index, 1)
    }

    this.setState({
      openSelected
    }, () => this.props.onOpenChange(openSelected))
  }

  renderItem = (arr) => {
    const { openSelected, itemSelected } = this.state
    return arr.map(item => {
      if (item.sub && item.sub.length) {
        return (
          <SubMenu key={item.name} name={item.name}
            eventKey={item.name}
            openSelected={openSelected}
            onOpenChange={this.onOpenChange}
          >{this.renderItem(item.sub)}</SubMenu>
        )
      } else {
        return (
          <Item key={item.name} name={item.name}
            eventKey={item.name}
            route={item.route}
            itemSelected={itemSelected}
            onItemChange={this.onItemChange}>{item.name}</Item>
        )
      }
    })
  }

  componentWillUpdate(nextProps, nextState) {
    if (this.props.location.pathname !== nextProps.location.pathname) {
      this.setState({
        openSelected: nextProps.location.state.openSelected ? nextProps.location.state.openSelected : [],
        itemSelected: [nextProps.location.state.itemSelected]
      })
    }
  }

  renderMenu = () => {
    const menu = this.props.menu
    let list = this.renderItem(menu.expands)

    return list
  }

  render () {

    return (
      <div id="setup-menu">
        <ul className="expand">
          {this.renderMenu()}
        </ul>
      </div>
    )
  }
}

SetupMenu.propTypes = {
  menu: PropTypes.object.isRequired,
  defaultSelected: PropTypes.string.isRequired
}

export default SetupMenu
