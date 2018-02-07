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
import ReplayList from '../component/ReplayList'
import NavList from '../component/NavList'
import FileList from '../component/FileList'

class ReplayContainer extends Component {
  constructor (props) {
    super(props)
    this.state = {
      listSelected: ['video']
    }
  }

  onChange = e => {
    this.setState({
      listSelected: e.target
    })
  }

  choseFile = e => {
    this.refs.replayArea.innerHTML = ''
    if (e.type === 'video') {
      let div = document.createElement('div')
      div.id = 'player'
      div.style.height = '540px'
      div.style.width = '960px'
      this.refs.replayArea.appendChild(div)
      $("#player").html('<video width="960" height="540" controls> <source src="'+e.url+'"  type="video/mp4">Your browser does not support HTML5 video tags. </video>')
    } else {
      let img = document.createElement('img')
      img.src = e.url
      img.style.maxWidth = '100%'
      img.style.maxHeight = '100%'
      let div = document.createElement('div')
      div.id = 'image'
      div.style.height = '50%'
      div.style.width = '50%'
      div.appendChild(img)
      this.refs.replayArea.appendChild(div)
    }
  }
  deleteSuccess = () => {
    this.refs.replayArea.innerHTML = ''
  }
    render () {
    const { listSelected } = this.state

    return (
      <Layout>
        <ReplayList>
          <NavList selected={listSelected} onChange={this.onChange}></NavList>
          <FileList selected={listSelected} onClick={this.choseFile} deleteSuccess={this.deleteSuccess}></FileList>
        </ReplayList>
        <div id="replay-area" ref="replayArea" style={{ flex: 'auto' }}>
        </div>
      </Layout>
    )
  }
}

export default ReplayContainer
