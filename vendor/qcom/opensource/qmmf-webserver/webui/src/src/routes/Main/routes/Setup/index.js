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

import SetupContainer from './container'
import CameraVideoRoute from './routes/CameraVideo'
import LanAddressRoute from './routes/LanAddress'
import wifiAddressRoute from './routes/WifiAddress'
import ReplayRoute from './routes/Replay'
import SensitivityRoute from './routes/Sensitivity'

export default {
  path: 'setup',
  component: SetupContainer,
  onEnter (nextState, replace) {
    let target = nextState.location.pathname
    if (target === 'setup' || target === '/setup') {
      console.log({ pathname: '/setup/cameraVideo', state: {openSelected: ['Camera Settings'],
        itemSelected: '1.Video'}})
      replace({ pathname: '/setup/cameraVideo', state: {openSelected: ['Camera Settings'],
        itemSelected: '1.Video'}})
    }
  },
  childRoutes: [
    CameraVideoRoute,
    LanAddressRoute,
    wifiAddressRoute,
    ReplayRoute,
    SensitivityRoute
  ]
}
