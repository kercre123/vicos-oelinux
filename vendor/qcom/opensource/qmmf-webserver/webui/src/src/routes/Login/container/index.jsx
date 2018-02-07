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
import { hashHistory } from 'react-router'
import ErrorLabel from '../component/ErrorLabel'
import { sendLogin } from 'src/ajax'
import { connect } from 'react-redux'
import { initUser } from 'src/store/user'
import { initSocket, sendFirstKeepAlive, sendIntervalKeepAlive } from 'src/store/socket'
import { getCookie } from 'src/util'
import { uri } from 'src/settings'
import layer from 'layerUI'

class LoginContainer extends Component {

  constructor (props) {
    super(props)
    this.tag = 'Login Container'
    this.loading = false
    this.state = {
      username: '',
      showUsernameError: '',
      password: '',
      showPasswordError: '',
      error: false
    }
  }

  login = () => {
    this.setState({
      showUsernameError: '',
      showPasswordError: ''
    }, this.loginCallback)
  }

  loginCallback = () => {
    this.error = false
    const username = this.refs['username'].value.trim()
    const password = this.refs['password'].value.trim()
    if (username === '') {
      this.error = true
      this.setState({
        showUsernameError: 'required'
      })
    }
    if (password === '') {
      this.error = true
      this.setState({
        showPasswordError: 'required'
      })
    }
    if (this.loading) {
      return
    }

    if (!this.error) {
      this.loading = true
      let socket
      if (__DEV__) {
        socket = new WebSocket('ws://localhost:8003')
      } else {
        socket = new WebSocket(uri.ws)
      }
      this.props.initSocket(socket).then(status => {
        if (status === 'open') {
          return sendLogin(username, password)
        }
      }).then(data => {
        this.loading = false
        if (data.status) {
          this.props.initUser('admin')
          this.props.sendFirstKeepAlive({ type : 'keep-alive', cookie: getCookie() })
          this.props.sendIntervalKeepAlive()
          window.localStorage.setItem("init_status", 1)
          hashHistory.push('/')
        } else {
          layer.msg('Please input right User ID and Password.', {
              icon : 5
          })
        }
      }).catch(error => {
        this.loading = false
        console.log(error)
      })
    }
  }

  render () {
    let { username, password } = this.state
    return (
      <div id="login_body" className="body">
        <div id="login_form">
          <div className="form-control">
            <input type="text" id="username" maxLength="24" placeholder="User" ref="username" autoComplete="off"/>
            <ErrorLabel htmlFor="username" showError={this.state.showUsernameError}></ErrorLabel>
          </div>
          <div className="form-control">
            <input type="password" id="password" maxLength="24" placeholder="Password" ref="password" autoComplete="off"/>
            <ErrorLabel htmlFor="password" showError={this.state.showPasswordError}></ErrorLabel>
          </div>
          <div>
            <button onClick={this.login}>Login</button>
          </div>
        </div>
      </div>
    )
  }
}

const mapStateToProps = state => ({
  user: state.user
})

const mapDispatchToProps = {
  initUser,
  initSocket,
  sendFirstKeepAlive,
  sendIntervalKeepAlive
}


export default connect(mapStateToProps, mapDispatchToProps)(LoginContainer)
