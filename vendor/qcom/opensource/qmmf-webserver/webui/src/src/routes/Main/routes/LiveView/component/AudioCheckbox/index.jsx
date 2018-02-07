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
import Checkbox from 'src/component/Checkbox'
import { changePaddingStatus} from 'store/padding'
import { changeAudioStatus } from 'store/live'

class AudioCheckbox extends Component {
    onClick = () => {
        const { padding, audioStatus } = this.props
        if (padding) {
            return
        }
        console.log("++++++++++++"+audioStatus)
        if (audioStatus) {
            return this.uncheck()
        }
        return this.check()
    }

    check = async () => {
        const { changePaddingStatus, video, changeAudioStatus } = this.props
        try {
            changePaddingStatus(true)
            changeAudioStatus(true)
            video.muted = false
        } catch (error) {
            !error.deal && layer.msg('Open Onvif Fail！Please try again later.', { icon: 5 })
        } finally {
            changePaddingStatus(false)
        }
    }

    uncheck = async () => {
        const { changePaddingStatus, video, changeAudioStatus } = this.props
        try {
            changePaddingStatus(true)
            changeAudioStatus(false)
            video.muted = true
        } catch (error) {
            !error.deal && layer.msg('Close Onvif Fail! Please try again later.', { icon: 5 })
        } finally {
            changePaddingStatus(false)
        }
    }
    render () {
        const { audioStatus } = this.props
        const props = {
            checked: audioStatus,
            name: 'Enable Audio',
            onClick: this.onClick
        }

        return (
            <Checkbox {...props}></Checkbox>
        )
    }
}

const mapStateToProps = state => ({
    audioStatus: state.live.audioStatus,
    video: state.video,
    padding: state.padding
})

const mapDispatchToProps = {
    changeAudioStatus,
    changePaddingStatus
}

export default connect(mapStateToProps, mapDispatchToProps)(AudioCheckbox)

