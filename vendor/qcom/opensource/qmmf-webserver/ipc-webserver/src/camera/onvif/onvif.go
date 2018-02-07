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

package onvif

import (
    "net/http"
    "os"
    ."common"
    "encoding/json"
)

// resp json
type Resp_Switch struct {
    SwitchStatus bool `json:"switchStatus"`
}

func __switch_onvif_on(w http.ResponseWriter) {
    f, err := os.Create(ONVIF_FLAG_FILE)

    if err != nil {
        Resp_ret(w, false)
        return
    }

    f.Sync()

    Resp_ret(w, true)
}

func __switch_onvif_off(w http.ResponseWriter) {
    err := os.Remove(ONVIF_FLAG_FILE)

    if err != nil {
        Resp_ret(w, false)
        return
    }

    Resp_ret(w, true)
}

func __onvif_post(w http.ResponseWriter, r *http.Request) {
    st, _ := Get_switch_status(r)

    if st {
        __switch_onvif_on(w)
    } else {
        __switch_onvif_off(w)
    }
}

func __resp_onvif(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    w.Write(j)
}

func __onvif_get(w http.ResponseWriter) {
    OnvifSt = false

    if Exist(ONVIF_FLAG_FILE) {
        OnvifSt = true
    }

    m := &Resp_Switch{
        SwitchStatus: OnvifSt,
    }
    __resp_onvif(w, m)
}

func OnvifHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "GET" {
        __onvif_get(w)
    } else if r.Method == "POST" {
        __onvif_post(w, r)
    }
}
