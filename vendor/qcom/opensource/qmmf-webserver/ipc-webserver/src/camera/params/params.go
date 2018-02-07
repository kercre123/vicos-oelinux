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

package params

import (
    . "common"
    "encoding/json"
    "github.com/bitly/go-simplejson"
    "io/ioutil"
    "log"
    "net/http"
    "net/url"
    "strings"
    . "system/smap"
    "strconv"
)

const (
    __set_camera_param_URL string = "http://127.0.0.1:4000/setcameraparam"
)

type Params_conf struct {
    Params Cparams
}

var ParamsConf = Params_conf{
    Cparams{"off", "off"},
}

// resp json
type RespParams struct {
    Tnr    string `json:"TNR"`
    Shdr   string `json:"SHDR"`
    Status bool   `json:"status"`
}

func __resp_params(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    w.Write(j)
}

func __save_nr_params(nr string) {
    ParamsConf.Params.Tnr = nr
}

func __save_hdr_params(hdr string) {
    ParamsConf.Params.Shdr = hdr
}

func __params_post_tnr(tnr string) bool {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))

    if tnr == "on" {
        v.Add("nr_mode", "high-quality")
    } else {
        v.Add("nr_mode", tnr)
    }

    log.Printf("Params post form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__set_camera_param_URL, body)
    return Get_resp_ret(resp)
}

func __params_post_shdr(hdr string) bool {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))

    v.Add("hdr_mode", hdr)

    log.Printf("Params post form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__set_camera_param_URL, body)
    return Get_resp_ret(resp)
}

func __params_post(w http.ResponseWriter, r *http.Request, chinfo Channel) {
    chState := chinfo.Status

    // check state
    if chState == Normal {
        Resp_ret_reason(w, "Please open preview or rec")
        return
    }

    body, err := ioutil.ReadAll(r.Body)
    if err != nil {
        panic("error")
    }

    if len(body) == 0 {
        log.Printf("Error tnr shdr is null\n")
        return
    }

    js, err := simplejson.NewJson(body)

    tnr, _ := js.Get("TNR").String()
    hdr, _ := js.Get("SHDR").String()

    if strings.Compare(tnr, ParamsConf.Params.Tnr) != 0 {
        if !__params_post_tnr(tnr) {
            Resp_ret_reason(w, "Internal error")
            return
        }
        __save_nr_params(tnr)
    }

    if strings.Compare(hdr, ParamsConf.Params.Shdr) != 0 {
        if !__params_post_shdr(hdr) {
            Resp_ret_reason(w, "Internal error")
            return
        }
        __save_hdr_params(hdr)
    }

    Resp_ret(w, true)
}

func __params_get(w http.ResponseWriter) {
    // resp params json
    m := &RespParams{
        Tnr:    ParamsConf.Params.Tnr,
        Shdr:   ParamsConf.Params.Shdr,
        Status: true,
    }

    __resp_params(w, m)
}

func ParamsHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    // get session channel
    ch := sessinfo.Channel
    // get channel info
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    if r.Method == "POST" {
        __params_post(w, r, chinfo)
        return
    }

    if r.Method == "GET" {
        __params_get(w)
        return
    }
}
