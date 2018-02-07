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
import { Link } from 'react-router'
import { NAVS } from 'settings'
import './header.sass'

const mapStateToProps = state => ({
  padding: state.padding,
  location: state.location,
})

class Header extends Component {
  constructor (props) {
    super(props)
    this.state = {
      showOptions: false,
      navs: NAVS
    }
  }

  onClick = pathname => {
    if (this.props.padding) {
      return
    }
    hashHistory.push({ pathname })
  }

  renderLink = navs => {
    return navs.map(nav => {
      const re = new RegExp(nav.path)
      const className = re.test(this.props.location.hash) ? 'nav highlight' : 'nav'
      const props = {
        className,
        onClick: () => this.onClick(nav.path),
        key: nav.path,
      }
      return <a {...props}>{nav.name}</a>
    })
  }

  render () {
    const { showOptions, navs } = this.state

    return (
      <div id="header">
        <div className="header-left">
          <div className="logo-1"></div>
          <div className="logo-2"></div>
        </div>
        <div className="header-right">
          <div id="navs">
            {this.renderLink(navs)}
          </div>
          <div id="user">
            <div id="user-name">
              <span>Admin</span>
              <span id="get-more-option"></span>
                <div id="user-option-group">
                <div className="user-option" onClick={this.props.logout}>Logout</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    )
  }
}

Header.propTypes = {
  logout: PropTypes.func.isRequired
}

export default connect(mapStateToProps)(Header)
