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
import SlideBtn from 'src/component/SlideBtn'
import { changePaddingStatus} from 'store/padding'
import { openOnvif, closeOnvif } from 'store/onvif'

class Onvif extends Component {
    onClick = () => {
        const { padding, onvifSwitchStatus } = this.props
        if (padding) {
            return
        }
        if (onvifSwitchStatus) {
            return this.close()
        }
        return this.open()
    }

    open = async () => {
        const { changePaddingStatus, openOnvif } = this.props
        try {
            changePaddingStatus(true)
            await openOnvif()
            if (this.props.onvifSwitchStatus) {
                $("select").attr("disabled","disabled")
                layer.msg('Open Onvif success! Please reboot device to use.', { icon: 6, area: '400px' })
            } else {
                layer.msg('Open Onvif Fail！Please try again later.', { icon: 5 })
            }
        } catch (error) {
            !error.deal && layer.msg('Open Onvif Fail！Please try again later.', { icon: 5 })
        } finally {
            changePaddingStatus(false)
        }
    }

    close = async () => {
        const { changePaddingStatus, closeOnvif } = this.props
        try {
            changePaddingStatus(true)
            await closeOnvif()
            if (!this.props.onvifSwitchStatus) {
                $("select").removeAttr("disabled")
                layer.msg('Close Onvif success! ', { icon: 6 })
            } else {
                layer.msg('Close Onvif Fail! Please try again later.', { icon: 5 })
            }
        } catch (error) {
            !error.deal && layer.msg('Close Onvif Fail! Please try again later.', { icon: 5 })
        } finally {
            changePaddingStatus(false)
        }
    }
    render () {
        const { onvifSwitchStatus } = this.props
        const props = {
            open: onvifSwitchStatus,
            name: 'Onvif',
            onClick: this.onClick
        }

        return (
            <SlideBtn {...props}></SlideBtn>
        )
    }
}

const mapStateToProps = state => ({
    onvifSwitchStatus: state.onvif.switchStatus,
    padding: state.padding
})

const mapDispatchToProps = {
    openOnvif,
    closeOnvif,
    changePaddingStatus
}

export default connect(mapStateToProps, mapDispatchToProps)(Onvif)

