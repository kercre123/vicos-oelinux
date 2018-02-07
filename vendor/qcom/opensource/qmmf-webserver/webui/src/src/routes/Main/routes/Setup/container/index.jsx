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
import { hashHistory } from 'react-router'
import Layout from '../component/Layout'
import SetupMenu from '../component/SetupMenu'
import { locationChange } from 'store/location'

class SetupContainer extends Component {
  constructor (props) {
    super(props)
    this.state = {
      menu: {
        expands: [
          {
            name: 'System Settings',
            sub: [
              {
                name: 'Network',
                sub: [
                  {
                    name: '1.LAN',
                    route: 'lanAddress'
                  },
                  {
                    name: '2.WIFI',
                    route: 'wifiAddress'
                  }
                ]
              }
            ]
          },
          {
            name: 'Camera Settings',
            sub: [
              {
                name: '1.Video',
                route: 'cameraVideo'
              }
            ]
          },
          {
            name: 'Replay',
            route: 'replay'
          },
          {
            name: 'Video Analysis Setting',
            route: 'sensitivity'
          }
        ],
      }
    }
  }

  onSelected = (e) => {
    hashHistory.push({ pathname: `/setup/${e.route}`, state: {openSelected: this.openSelected,
      itemSelected:e.eventKey }})
  }

  onOpenChange = openSelected => {
    this.openSelected = openSelected
  }

  componentWillMount () {
    this.defaultSelected = this.props.location.state.itemSelected
    this.defaultOpenSelected = this.props.location.state.openSelected
  }

  render () {
    const { menu } = this.state
    return (
      <Layout>
        <SetupMenu menu={menu}
          onItemClick={this.onSelected}
          defaultSelected={this.defaultSelected}
          defaultOpenSelected={this.defaultOpenSelected}
          onOpenChange={this.onOpenChange}
          location={this.props.location}></SetupMenu>
        <div id="setup-main-area">
          {this.props.children}
        </div>
      </Layout>
    )
  }
}

export default SetupContainer
