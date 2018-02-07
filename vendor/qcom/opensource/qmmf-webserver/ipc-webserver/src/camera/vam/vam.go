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

package vam

import (
    . "common"
    "encoding/json"
    "io/ioutil"
    "log"
    "net/http"
    "net/url"
    "strconv"
    "strings"
    . "system/smap"
)

const (
    __crate_video_track_URL string = "http://127.0.0.1:4000/createvideotrack"
    __vam_config_URL        string = "http://127.0.0.1:4000/vamconfig"
    __vam_remove_config_URL string = "http://127.0.0.1:4000/vamremoveconfig"
    __vam_enroll_URL        string = "http://127.0.0.1:4000/vamenroll"
)

type RespVamURL struct {
    Url     string `json:"url"`
    FrProxy string `json:"fr_port"`
    CtProxy string `json:"ct_port"`
    MdProxy string `json:"md_port"`
    Status  bool   `json:"status"`
}

type EnrollIdParams struct {
    VamEnrollId int `json:"enroll_id"`
}

var __conf_state = map[string]int{
    "FR": FR,
    "CT": CT,
    "MD": MD,
    "OC": OC,
}

func __create_vam_video_track(chinfo Channel) bool {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))
    v.Add("track_id", strconv.Itoa(chinfo.TrackIds.VamId))
    v.Add("session_id", strconv.Itoa(chinfo.Sids.VamId))
    v.Add("track_width", strconv.Itoa(VAMResolution.Width))
    v.Add("track_height", strconv.Itoa(VAMResolution.Height))
    v.Add("framerate", "30")
    v.Add("track_codec", "2")
    // input vam
    v.Add("track_output", "1")
    v.Add("bitrate", "0")
    v.Add("low_power_mode", "0")

    log.Printf("Vam createvideotrack form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__crate_video_track_URL, body)
    return Get_resp_ret(resp)
}

func __vam_on(chinfo Channel) bool {
    if !__create_vam_video_track(chinfo) {
        return false
    }

    if !Start_session(chinfo.Sids.VamId) {
        return false
    }
    return true
}

func __vam_off(chinfo Channel) bool {
    if !Stop_session(chinfo.Sids.VamId) {
        return false
    }

    if !Delete_video_track(chinfo.TrackIds.VamId, chinfo.Sids.VamId) {
        return false
    }
    return true
}

func __switch_vam_on(w http.ResponseWriter, vamconf string, sess string,
    chinfo Channel, ch int) {
    sessinfo, _ := SMap.Get(sess)

    // check state
    if !sessinfo.PreStatus {
        Resp_ret_reason(w, "Please open preview first")
        return
    }

    if chinfo.VCounter != 0 {
        __update_session_vam_st(__conf_state[vamconf], sess, true)
        // VCounter + 1
        chinfo.VCounter++
        CMap.ChMap_Set(ChList[ch], chinfo)

        Resp_ret(w, true)
        return
    }

    if !__vam_on(chinfo) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    chinfo.VCounter += 2
    //chinfo.Status = Vam
    CMap.ChMap_Set(ChList[ch], chinfo)

    __update_session_vam_st(__conf_state[vamconf], sess, true)

    Resp_ret(w, true)
}

func Switch_vam_off(chinfo Channel, sess string, ch int) {
    if chinfo.VCounter != 1 {
        chinfo.VCounter--
        CMap.ChMap_Set(ChList[ch], chinfo)

        for index := FR; index < MAX; index++ {
            __update_session_vam_st(index, sess, false)
        }

        return
    }

    if !__vam_off(chinfo) {
        return
    }

    chinfo.VCounter--
    CMap.ChMap_Set(ChList[ch], chinfo)

    for index := FR; index < MAX; index++ {
        Close_rtsp_proxy(index, sess)
        __update_session_vam_st(index, sess, false)
    }
}

func __switch_vam_off(w http.ResponseWriter, vamconf string, sess string,
    chinfo Channel, ch int) {
    if chinfo.VCounter != 1 {
        __update_session_vam_st(__conf_state[vamconf], sess, false)

        chinfo.VCounter--
        CMap.ChMap_Set(ChList[ch], chinfo)

        Resp_ret(w, true)
        return
    }

    if !__vam_off(chinfo) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    chinfo.VCounter--
    CMap.ChMap_Set(ChList[ch], chinfo)

    __update_session_vam_st(__conf_state[vamconf], sess, false)

    Close_rtsp_proxy(__conf_state[vamconf], sess)

    Resp_ret(w, true)
}

func __vam_post(w http.ResponseWriter, r *http.Request, sess string,
    chinfo Channel, ch int) {

    st, conf := Get_switch_status(r)

    if st {
        __switch_vam_on(w, conf, sess, chinfo, ch)
    } else {
        __switch_vam_off(w, conf, sess, chinfo, ch)
    }
}

func __update_session_vam_st(conf int, sess string, status bool) {
    sessinfo, _ := SMap.Get(sess)
    sessinfo.ProxyConf[conf].Status = status
    SMap.Set(sess, sessinfo)
}

func __vam_get(w http.ResponseWriter, r *http.Request, sess string,
    sessinfo Session, chinfo Channel) {

    log.Printf("Vam get channel vcounter:%v \n", chinfo.VCounter)
    if chinfo.VCounter == 0 {
        Resp_ret(w, false)
        return
    }

    vam_url := Get_server_info(chinfo.Sids.VamId)
    log.Printf("Vam get rtsp url:%v \n", vam_url)

    if vam_url == "" {
        log.Printf("Get rtsp url is nil!!!")
        Resp_ret_reason(w, "Internal error")
        return
    }

    vamURL := vam_url
    if IsWifiAPMode {
        vamURL = Make_resp_rtspURL(CameraIp, vam_url)
    }

    log.Printf("Vam resp rtsp url:%v \n", vamURL)

    if r.URL.RawQuery == "" {
        m := &RespVamURL{
            Url:     vamURL,
            FrProxy: sessinfo.ProxyConf[FR].Port,
            CtProxy: sessinfo.ProxyConf[CT].Port,
            MdProxy: sessinfo.ProxyConf[MD].Port,
            Status:  true,
        }

        Resp_url(w, m)
    } else {
        k, _ := url.ParseQuery(r.URL.RawQuery)

        v := k["vamconfig"][0]

        port := Set_rtsp_proxy(__conf_state[v], sess, sessinfo)
        log.Printf("Get proxy port:%v \n", port)

        // get new sessinfo
        sessinfo, _ = SMap.Get(sess)

        m := &RespRtspURL{
            Url:    vamURL,
            Proxy:  sessinfo.ProxyConf[__conf_state[v]].Port,
            Status: true,
        }

        Resp_url(w, m)
    }

}

func VamHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    chinfo, ch := Vam_tmp_one_streaming(sessinfo)

    if r.Method == "POST" {
        __vam_post(w, r, sess, chinfo, ch)
    }

    if r.Method == "GET" {
        if !sessinfo.ProxyConf[FR].Status &&
            !sessinfo.ProxyConf[CT].Status &&
            !sessinfo.ProxyConf[MD].Status &&
            !sessinfo.ProxyConf[OC].Status {
            Resp_ret(w, false)
            return
        }
        __vam_get(w, r, sess, sessinfo, chinfo)
    }
}

func __query_vam_config_type(rawQuery string) int {
    var config string

    if rawQuery == "" {
        return -1
    } else {
        k, _ := url.ParseQuery(rawQuery)

        config = k["type"][0]
    }

    return __conf_state[config]
}

func __vam_config_post(w http.ResponseWriter, r *http.Request,
    chinfo Channel, ch int) {

    rawQuery := r.URL.RawQuery
    log.Printf("RawQuery:%v \n", rawQuery)

    vamtype := __query_vam_config_type(rawQuery)

    log.Printf("Vam config post type:%v \n", vamtype)

    if vamtype != OC {
        if chinfo.VamConfig[vamtype].Status {
            chinfo.VamConfig[vamtype].Counter++

            CMap.ChMap_Set(ChList[ch], chinfo)

            Resp_ret(w, true)
            return
        }
    }

    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    log.Printf("Vam config post:%v \n", string(body))

    data := ioutil.NopCloser(strings.NewReader(string(body)))

    resp := Http_post(__vam_config_URL, data)

    if !Get_resp_ret(resp) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    // set vaconfig status
    chinfo.VamConfig[vamtype].Counter++
    chinfo.VamConfig[vamtype].Status = true
    chinfo.VamConfig[vamtype].Config = string(body)
    CMap.ChMap_Set(ChList[ch], chinfo)

    Resp_ret(w, true)
}

func VamConfigHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    chinfo, ch := Vam_tmp_one_streaming(sessinfo)

    if r.Method == "POST" {
        __vam_config_post(w, r, chinfo, ch)
    }
}

func Vam_remove_config(chinfo Channel, ch int, vamtype int) {
    if vamtype != OC {
        if chinfo.VamConfig[vamtype].Status &&
        chinfo.VamConfig[vamtype].Counter != 1 {
            chinfo.VamConfig[vamtype].Counter--

            CMap.ChMap_Set(ChList[ch], chinfo)
            return
        }
    }

    data := ioutil.NopCloser(strings.NewReader(chinfo.VamConfig[vamtype].Config))

    log.Printf("Vam remove config post:%v \n", chinfo.VamConfig[vamtype].Config)

    resp := Http_post(__vam_remove_config_URL, data)

    if !Get_resp_ret(resp) {
        return
    }

    // set vaconfig status
    chinfo.VamConfig[vamtype].Counter--
    chinfo.VamConfig[vamtype].Status = false
    chinfo.VamConfig[vamtype].Config = ""
    CMap.ChMap_Set(ChList[ch], chinfo)
}

func __vam_remove_config_post(w http.ResponseWriter, r *http.Request,
    chinfo Channel, ch int) {

    vamtype := __query_vam_config_type(r.URL.RawQuery)

    log.Printf("Vam remove config post type:%v \n", vamtype)

    if vamtype != OC {
        if chinfo.VamConfig[vamtype].Status &&
        chinfo.VamConfig[vamtype].Counter != 1 {
            chinfo.VamConfig[vamtype].Counter--

            CMap.ChMap_Set(ChList[ch], chinfo)

            Resp_ret(w, true)
            return
        }
    }

    body, err := ioutil.ReadAll(r.Body)
    if err != nil {
        panic("error")
    }

    log.Printf("Vam remove config post:%v \n", string(body))

    data := ioutil.NopCloser(strings.NewReader(string(body)))

    resp := Http_post(__vam_remove_config_URL, data)

    if !Get_resp_ret(resp) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    // set vaconfig status
    chinfo.VamConfig[vamtype].Counter--
    chinfo.VamConfig[vamtype].Status = false
    chinfo.VamConfig[vamtype].Config = ""
    CMap.ChMap_Set(ChList[ch], chinfo)

    Resp_ret(w, true)
}

func VamRemoveConfigHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    chinfo, ch := Vam_tmp_one_streaming(sessinfo)

    if r.Method == "POST" {
        __vam_remove_config_post(w, r, chinfo, ch)
    }
}

func __save_enroll_id() {
    m := &EnrollIdParams{
        VamEnrollId: EnrollId,
    }

    j, _ := json.Marshal(m)

    Save_file(ENROLL_ID_FILE, j)
}

func __vam_enroll_post(w http.ResponseWriter, r *http.Request, chinfo Channel) {
    // check state
    log.Printf("Vam get channel vcounter:%v \n", chinfo.VCounter)

    if chinfo.VCounter == 0 {
        Resp_ret_reason(w, "Please open FR first")
        return
    }

    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    // id + 1
    EnrollId++

    enroll_params := string(body) + "&" + "vam_enroll_image_id=" + strconv.Itoa(EnrollId)

    data := ioutil.NopCloser(strings.NewReader(enroll_params))

    resp := Http_post(__vam_enroll_URL, data)

    if !Get_resp_ret(resp) {
        // If fail -1
        EnrollId--
        Resp_ret_reason(w, "Internal error")
        return
    }
    // save file
    __save_enroll_id()

    Resp_ret(w, true)
}

func VamEnrollHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    chinfo, _ := Vam_tmp_one_streaming(sessinfo)

    if r.Method == "POST" {
        if !sessinfo.ProxyConf[FR].Status {
            Resp_ret(w, false)
            return
        }
        __vam_enroll_post(w, r, chinfo)
    }
}

func Vam_tmp_one_streaming(sessinfo Session) (Channel, int) {
    var chinfo Channel
    var opened bool = false
    var ch int

    for index := 0; index < MaxChannelNum; index++ {
        chinfo, _ = CMap.ChMap_Get(ChList[index])
        if chinfo.VCounter != 0 {
            ch = index
            opened = true
            break
        }
    }

    if !opened {
        // get session channel
        ch = sessinfo.Channel
        // get channel info
        chinfo, _ = CMap.ChMap_Get(ChList[ch])
    }

    return chinfo, ch
}
