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
import SlideBtn from 'src/component/SlideBtn'
import  'antd/lib/slider/style/index.css'
import { Slider, Switch } from 'antd';
import { Input } from 'antd';
import layer from 'layerUI'
import { LOCAL_STORE_KEY } from 'src/settings'
import { getExistDataByKey, setExistDataByKey } from 'src/util'

const FR_CONFIT_KEY = LOCAL_STORE_KEY.fr_config
const MD_CONFIT_KEY = LOCAL_STORE_KEY.md_config
const OB_CONFIT_KEY = LOCAL_STORE_KEY.ob_config
const CT_CONFIT_KEY = LOCAL_STORE_KEY.ct_config
const MD_AND_CT_CONFIG_KEY = LOCAL_STORE_KEY.md_and_ct_config

class sensitivityContainer extends Component {
    constructor (props) {
        super(props)
        let faceRecognitionConfig = getExistDataByKey(FR_CONFIT_KEY)
        if (!faceRecognitionConfig) {
            faceRecognitionConfig = {
                'atomic_rules': [
                    {
                        'sensitivity':62
                    }
                ]
            }
        }

        let motionDetectionConfig = getExistDataByKey(MD_CONFIT_KEY)
        if (!motionDetectionConfig) {
            motionDetectionConfig = {
                'atomic_rules':[
                    {
                        'sensitivity': 70,
                    }
                ]
            }
        }
        let miniSizeConfig = getExistDataByKey(OB_CONFIT_KEY)
        if (!miniSizeConfig) {
            miniSizeConfig = {
                'atomic_rules':[
                    {
                        'sensitivity': 10
                    }
                ]
            }
        }
        let cameraTamperConfig = getExistDataByKey(CT_CONFIT_KEY)
        if (!cameraTamperConfig) {
            cameraTamperConfig = {
                'atomic_rules':[
                    {
                        'sensitivity': 1
                    }
                ]
            }
        }
        let mdAndCtConfig = getExistDataByKey(MD_AND_CT_CONFIG_KEY)
        if (!mdAndCtConfig) {
            mdAndCtConfig = {
                isOpen: false
            }
        }
        this.state = {
            FR : faceRecognitionConfig,
            MD : motionDetectionConfig,
            MZ : miniSizeConfig,
            CT : cameraTamperConfig,
            mdAndCt : mdAndCtConfig
        }
    }
    setSensitivity = async () => {
        const {FR, MD, MZ, CT, mdAndCt} = this.state
        try{
            setExistDataByKey(FR_CONFIT_KEY, FR)
            setExistDataByKey(MD_CONFIT_KEY, MD)
            setExistDataByKey(OB_CONFIT_KEY, MZ)
            setExistDataByKey(CT_CONFIT_KEY, CT)
            setExistDataByKey(MD_AND_CT_CONFIG_KEY, mdAndCt)
            let msg, icon
            msg = 'VAM Sensitivity Setting success！'
            icon = 6
            layer.msg(msg, {icon})
        } catch (error) {
            console.log(error)
            let msg, icon
            msg = 'VAM Sensitivity Setting Fail！Please try again later.'
            icon = 5
            layer.msg(msg, {icon})
        }
    }

    componentWillMount () {

    }
    onChange = e => {
        this.state.FR.atomic_rules[0].sensitivity = e
        this.setState({
            FR: this.state.FR
        })
    }
    onChange2 = e => {
        this.state.MD.atomic_rules[0].sensitivity = e
        this.setState({
            MD: this.state.MD
        })
    }
    onChange3 = e => {
        this.state.MZ.atomic_rules[0].sensitivity = e
        this.setState({
            MZ: this.state.MZ
        })
    }
    onChange4 = e => {
        this.state.CT.atomic_rules[0].sensitivity = e
        this.setState({
            CT: this.state.CT
        })
    }
    mdCtSlideBtnOnClick = e => {
        const {isOpen} = this.state.mdAndCt
        this.setState({
            mdAndCt: {
                isOpen: !isOpen
            }
        })
    }
    render () {
        const { FR, MD, MZ, CT, mdAndCt } = this.state
        const mdCtSlideBtnProps = {
            open: mdAndCt.isOpen,
            name: 'Motion Detection + Camera Tamper: ',
            onClick: this.mdCtSlideBtnOnClick,
            slideStyle: {
                'margin': '20px 0'
            }
        }
        return (
            <Layout>
                <div className="header">Video Analysis Setting</div>
                <div className="main">
                    Face Recognition Sensitivity: {FR.atomic_rules[0].sensitivity}
                    <Slider defaultValue={FR.atomic_rules[0].sensitivity} onChange={this.onChange} max={95}/>

                    <SlideBtn {...mdCtSlideBtnProps}></SlideBtn>

                    Motion Detection Sensitivity: {MD.atomic_rules[0].sensitivity}
                    <Slider defaultValue={MD.atomic_rules[0].sensitivity} onChange={this.onChange2} max={95}/>
                    Minimum Size of Object: {MZ.atomic_rules[0].sensitivity}
                    <Slider defaultValue={MZ.atomic_rules[0].sensitivity} onChange={this.onChange3} max={95}/>
                    Camera Tamper Sensitivity: {CT.atomic_rules[0].sensitivity}
                    <Slider defaultValue={CT.atomic_rules[0].sensitivity} onChange={this.onChange4} max={95}/>
                </div>
                <div className="footer">
                    <button className="primary"
                            onClick={this.setSensitivity}>
                        Save
                    </button>
                    <Input type="hidden" value="{FR.atomic_rules[0].sensitivity}" id="FR"/>
                    <Input type="hidden" value="{MD.atomic_rules[0].sensitivity}" id="MD"/>
                    <Input type="hidden" value="{MZ.atomic_rules[0].sensitivity}" id="MZ"/>
                    <Input type="hidden" value="{CT.atomic_rules[0].sensitivity}" id="CT"/>
                </div>
            </Layout>
        );
    }
}

export default sensitivityContainer