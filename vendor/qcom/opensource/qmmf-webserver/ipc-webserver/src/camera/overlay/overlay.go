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

package overlay

import (
    . "common"
    "encoding/json"
    "github.com/bitly/go-simplejson"
    "io/ioutil"
    "log"
    "net/http"
    "net/url"
    "strconv"
    "strings"
    . "system/smap"
    "os"
)

const (
    // qmmfwebserver url
    __create_overlay_URL string = "http://127.0.0.1:4000/createoverlay"
    __set_overlay_URL    string = "http://127.0.0.1:4000/setoverlay"

    __delete_overlay_URL string = "http://127.0.0.1:4000/deleteoverlay"
    __remove_overlay_URL string = "http://127.0.0.1:4000/removeoverlay"
)

// Ovtype list
var OvTypeList = []string{"usertext","datetime", "boundingbox", "staticimage", "privacymask"}

// OvPosition list
var OvPositionList = []string{"bottomright", "bottomleft", "topleft", "topright", "center"}

// OvDate list
var OvDateList = []string{"yyyymmdd", "mmddyyyy"}

// OvDate list
var OvTimeList = []string{"hhmmss_24hr", "hhmmss_ampm", "hhmm_24hr", "hhmm_ampm"}

// resp json
type Resp_Switch struct {
    SwitchStatus bool `json:"switchStatus"`
}

type Resp_Time struct {
    Ov_type_SelectVal     int      `json:"ov_type_SelectVal"`
    Ov_type               []string `json:"ov_type"`
    Ov_position_SelectVal int      `json:"ov_position_SelectVal"`
    Ov_position           []string `json:"ov_position"`
    Ov_date_SelectVal     int      `json:"ov_date_SelectVal"`
    Ov_date               []string `json:"ov_date"`
    Ov_time_SelectVal     int      `json:"ov_time_SelectVal"`
    Ov_time               []string `json:"ov_time"`
    Ov_color              string   `json:"ov_color"`
    Ov_start_x            int      `json:"ov_start_x"`
    Ov_start_y            int      `json:"ov_start_y"`
    Ov_width              int      `json:"ov_width"`
    Ov_height             int      `json:"ov_height"`
    Status                bool     `json:"status"`
}

type Resp_Text struct {
    Ov_type_SelectVal     int      `json:"ov_type_SelectVal"`
    Ov_type               []string `json:"ov_type"`
    Ov_position_SelectVal int      `json:"ov_position_SelectVal"`
    Ov_position           []string `json:"ov_position"`
    Ov_color              string   `json:"ov_color"`
    Ov_user_text          string   `json:"ov_usertext"`
    Ov_start_x            int      `json:"ov_start_x"`
    Ov_start_y            int      `json:"ov_start_y"`
    Ov_width              int      `json:"ov_width"`
    Ov_height             int      `json:"ov_height"`
    Status                bool     `json:"status"`
}

type Resp_Image struct {
    Ov_type_SelectVal     int      `json:"ov_type_SelectVal"`
    Ov_type               []string `json:"ov_type"`
    Ov_position_SelectVal int      `json:"ov_position_SelectVal"`
    Ov_position           []string `json:"ov_position"`
    Ov_width              int      `json:"ov_width"`
    Ov_height             int      `json:"ov_height"`
    Ov_image_location     int      `json:"ov_image_location"`
    Status                bool     `json:"status"`
}

type Resp_Box struct {
    Ov_type_SelectVal int      `json:"ov_type_SelectVal"`
    Ov_type           []string `json:"ov_type"`
    Ov_box_name       string   `json:"ov_box_name"`
    Ov_start_x        int      `json:"ov_start_x"`
    Ov_start_y        int      `json:"ov_start_y"`
    Ov_width          int      `json:"ov_width"`
    Ov_height         int      `json:"ov_height"`
    Ov_color          string   `json:"ov_color"`
    Status            bool     `json:"status"`
}

type Resp_Mask struct {
    Ov_type_SelectVal int      `json:"ov_type_SelectVal"`
    Ov_type           []string `json:"ov_type"`
    Ov_start_x        int      `json:"ov_start_x"`
    Ov_start_y        int      `json:"ov_start_y"`
    Ov_width          int      `json:"ov_width"`
    Ov_height         int      `json:"ov_height"`
    Ov_color          string   `json:"ov_color"`
    Status            bool     `json:"status"`
}

func __resp_overlay(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    w.Write(j)
}

func __overlay_config_get(w http.ResponseWriter, r *http.Request, chinfo Channel) {
    var ov_type int = 0

    log.Printf("Overlay config get:%v \n", r.URL.RawQuery)

    if r.URL.RawQuery == "" {
        ov_type = chinfo.OverlayType
    } else {
        k, _ := url.ParseQuery(r.URL.RawQuery)

        v := k["ov_type_SelectVal"][0]

        ov_type, _ = strconv.Atoi(v)
    }

    var m interface{}

    if ov_type == 0 {
        m = &Resp_Text{
            Ov_type_SelectVal:     ov_type,
            Ov_type:               OvTypeList[0:3],
            Ov_position_SelectVal: chinfo.OverlayConf[1].Ov_position,
            Ov_position:           OvPositionList,
            Ov_color:              chinfo.OverlayConf[1].Ov_color,
            Ov_user_text:          chinfo.OverlayConf[1].Ov_user_text,
            Ov_start_x:            chinfo.OverlayConf[1].Ov_start_x,
            Ov_start_y:            chinfo.OverlayConf[1].Ov_start_y,
            Ov_width:              chinfo.OverlayConf[1].Ov_width,
            Ov_height:             chinfo.OverlayConf[1].Ov_height,
            Status:                true,
        }
    } else if ov_type == 1 {
        m = &Resp_Time{
            Ov_type_SelectVal:     ov_type,
            Ov_type:               OvTypeList[0:3],
            Ov_position_SelectVal: chinfo.OverlayConf[0].Ov_position,
            Ov_position:           OvPositionList,
            Ov_date_SelectVal:     chinfo.OverlayConf[0].Ov_date,
            Ov_date:               OvDateList,
            Ov_time_SelectVal:     chinfo.OverlayConf[0].Ov_time,
            Ov_time:               OvTimeList,
            Ov_color:              chinfo.OverlayConf[0].Ov_color,
            Ov_start_x:            chinfo.OverlayConf[0].Ov_start_x,
            Ov_start_y:            chinfo.OverlayConf[0].Ov_start_y,
            Ov_width:              chinfo.OverlayConf[0].Ov_width,
            Ov_height:             chinfo.OverlayConf[0].Ov_height,
            Status:                true,
        }
    } else if ov_type == 2 {
        m = &Resp_Box{
            Ov_type_SelectVal: ov_type,
            Ov_type:           OvTypeList[0:3],
            Ov_box_name:       chinfo.OverlayConf[2].Ov_box_name,
            Ov_start_x:        chinfo.OverlayConf[2].Ov_start_x,
            Ov_start_y:        chinfo.OverlayConf[2].Ov_start_y,
            Ov_width:          chinfo.OverlayConf[2].Ov_width,
            Ov_height:         chinfo.OverlayConf[2].Ov_height,
            Ov_color:          chinfo.OverlayConf[2].Ov_color,
            Status:            true,
        }
    }

    __resp_overlay(w, m)
}

func __save_channel_overlay_conf(conf []byte, ch int, ov_type int) {

    f, err := os.Create(ChConfList[ch].Ov_Conf)

    if err != nil {
        panic("error")
    }
    defer f.Close()

    f.Write(conf)
    f.Sync()

    if ov_type == 0 {
        f, err = os.Create(ChConfList[ch].Ov_Text_Conf)
    } else if ov_type == 1 {
        f, err = os.Create(ChConfList[ch].Ov_Time_Conf)
    } else if ov_type == 2 {
        f, err = os.Create(ChConfList[ch].Ov_Box_Conf)
    }

    if err != nil {
        panic("error")
    }
    defer f.Close()

    f.Write(conf)
    f.Sync()
}

func __save_channel_overlay_map(body []byte, ch int, ov_type int) {
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    chinfo.OverlayType = ov_type

    if ov_type == 0 {
        jsdata, _ := simplejson.NewJson(body)

        chinfo.OverlayConf[1].Ov_type, _ = jsdata.Get("ov_type_SelectVal").Int()
        chinfo.OverlayConf[1].Ov_color, _ = jsdata.Get("ov_color").String()
        ov_position, _ := jsdata.Get("ov_position_SelectVal").Int()
        chinfo.OverlayConf[1].Ov_position = ov_position
        chinfo.OverlayConf[1].Ov_user_text, _ = jsdata.Get("ov_usertext").String()
        if ov_position == 5 { // If position is random
            chinfo.OverlayConf[1].Ov_start_x, _ = jsdata.Get("ov_start_x").Int()
            chinfo.OverlayConf[1].Ov_start_y, _ = jsdata.Get("ov_start_y").Int()
            chinfo.OverlayConf[1].Ov_width, _ = jsdata.Get("ov_width").Int()
            chinfo.OverlayConf[1].Ov_height, _ = jsdata.Get("ov_height").Int()
        }
    }

    if ov_type == 1 {
        jsdata, _ := simplejson.NewJson(body)

        chinfo.OverlayConf[0].Ov_type, _ = jsdata.Get("ov_type_SelectVal").Int()
        chinfo.OverlayConf[0].Ov_color, _ = jsdata.Get("ov_color").String()
        ov_position, _ := jsdata.Get("ov_position_SelectVal").Int()
        chinfo.OverlayConf[0].Ov_position = ov_position
        chinfo.OverlayConf[0].Ov_date, _ = jsdata.Get("ov_date_SelectVal").Int()
        chinfo.OverlayConf[0].Ov_time, _ = jsdata.Get("ov_time_SelectVal").Int()
        if ov_position == 5 { // If position is random
            chinfo.OverlayConf[0].Ov_start_x, _ = jsdata.Get("ov_start_x").Int()
            chinfo.OverlayConf[0].Ov_start_y, _ = jsdata.Get("ov_start_y").Int()
            chinfo.OverlayConf[0].Ov_width, _ = jsdata.Get("ov_width").Int()
            chinfo.OverlayConf[0].Ov_height, _ = jsdata.Get("ov_height").Int()
        }
    }

    if ov_type == 2 {
        jsdata, _ := simplejson.NewJson(body)

        chinfo.OverlayConf[2].Ov_type, _ = jsdata.Get("ov_type_SelectVal").Int()
        chinfo.OverlayConf[2].Ov_box_name, _ = jsdata.Get("ov_box_name").String()
        chinfo.OverlayConf[2].Ov_start_x, _ = jsdata.Get("ov_start_x").Int()
        chinfo.OverlayConf[2].Ov_start_y, _ = jsdata.Get("ov_start_y").Int()
        chinfo.OverlayConf[2].Ov_width, _ = jsdata.Get("ov_width").Int()
        chinfo.OverlayConf[2].Ov_height, _ = jsdata.Get("ov_height").Int()
        chinfo.OverlayConf[2].Ov_color, _ = jsdata.Get("ov_color").String()
    }
    // set Cmap
    CMap.ChMap_Set(ChList[ch], chinfo)
}

func __overlay_config_post(w http.ResponseWriter, r *http.Request, chinfo Channel, ch int) {
    // channel status

    if chinfo.OverlayStatus {
        Resp_ret_reason(w, "First turn off the overlay")
        return
    }

    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    jsdata, _ := simplejson.NewJson(body)

    ov_type, _ := jsdata.Get("ov_type_SelectVal").Int()
    // save config file
    __save_channel_overlay_conf(body, ch, ov_type)
    // save config map
    __save_channel_overlay_map(body, ch, ov_type)

    Resp_ret(w, true)
}

func OverlayConfigHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    if !ok {
        log.Printf("Overlay error sess not exist \n")
        Resp_ret(w, false)
        return
    }

    // get chinfo
    ch := sessinfo.Channel
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    if r.Method == "GET" {
        __overlay_config_get(w, r, chinfo)
    } else if r.Method == "POST" {
        __overlay_config_post(w, r, chinfo, ch)
    }
}

func __get_resp_ovid(resp *http.Response) (string, bool) {
    var ret = false

    body, err := ioutil.ReadAll(resp.Body)

    if err != nil {
        log.Printf("Server read body err:%v \n", err)
    }

    js, err := simplejson.NewJson(body)

    ovid, _ := js.Get("Status").String()
    e, _ := js.Get("Error").String()

    if strings.Compare(e, "none") == 0 {
        ret = true
    }

    defer resp.Body.Close()

    return ovid, ret
}

func __create_overlay(chinfo Channel) (string, bool) {
    v := url.Values{}

    if chinfo.Status == Preview {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RtspId))
    } else if chinfo.Status == Recording {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RecordingId))
    }

    v.Add("ov_type", OvTypeList[chinfo.OverlayType])

    if chinfo.OverlayType == 0 {
        v.Add("ov_position", OvPositionList[chinfo.OverlayConf[1].Ov_position])
        v.Add("ov_color", chinfo.OverlayConf[1].Ov_color)
        v.Add("ov_user_text", chinfo.OverlayConf[1].Ov_user_text)
        v.Add("ov_start_x", strconv.Itoa(chinfo.OverlayConf[1].Ov_start_x))
        v.Add("ov_start_y", strconv.Itoa(chinfo.OverlayConf[1].Ov_start_y))
        v.Add("ov_width", strconv.Itoa(chinfo.OverlayConf[1].Ov_width))
        v.Add("ov_height", strconv.Itoa(chinfo.OverlayConf[1].Ov_height))
    } else if chinfo.OverlayType == 1 {
        v.Add("ov_position", OvPositionList[chinfo.OverlayConf[0].Ov_position])
        v.Add("ov_color", chinfo.OverlayConf[0].Ov_color)
        v.Add("ov_date", OvDateList[chinfo.OverlayConf[0].Ov_date])
        v.Add("ov_time", OvTimeList[chinfo.OverlayConf[0].Ov_time])
        v.Add("ov_start_x", strconv.Itoa(chinfo.OverlayConf[0].Ov_start_x))
        v.Add("ov_start_y", strconv.Itoa(chinfo.OverlayConf[0].Ov_start_y))
        v.Add("ov_width", strconv.Itoa(chinfo.OverlayConf[0].Ov_width))
        v.Add("ov_height", strconv.Itoa(chinfo.OverlayConf[0].Ov_height))
    } else if chinfo.OverlayType == 2 {
        v.Add("ov_box_name", chinfo.OverlayConf[2].Ov_box_name)
        v.Add("ov_start_x", strconv.Itoa(chinfo.OverlayConf[2].Ov_start_x))
        v.Add("ov_start_y", strconv.Itoa(chinfo.OverlayConf[2].Ov_start_y))
        v.Add("ov_width", strconv.Itoa(chinfo.OverlayConf[2].Ov_width))
        v.Add("ov_height", strconv.Itoa(chinfo.OverlayConf[2].Ov_height))
        v.Add("ov_color", chinfo.OverlayConf[2].Ov_color)
    }
    /* } else if chinfo.OverlayConf.Ov_type == 3 {
        v.Add("ov_position", OvPositionList[chinfo.OverlayConf[2].Ov_position])
        v.Add("ov_width", strconv.Itoa(chinfo.OverlayConf[2].Ov_width))
        v.Add("ov_height", strconv.Itoa(chinfo.OverlayConf[2].Ov_height))
        v.Add("ov_image_location", strconv.Itoa(chinfo.OverlayConf.Ov_image_location))
    } else if chinfo.OverlayConf.Ov_type == 4 {
        v.Add("ov_start_x", strconv.Itoa(chinfo.OverlayConf.Ov_start_x))
        v.Add("ov_start_y", strconv.Itoa(chinfo.OverlayConf.Ov_start_y))
        v.Add("ov_width", strconv.Itoa(chinfo.OverlayConf.Ov_width))
        v.Add("ov_height", strconv.Itoa(chinfo.OverlayConf.Ov_height))
        v.Add("ov_color", strconv.Itoa(chinfo.OverlayConf.Ov_color))
    } */

    log.Printf("Preview create overlay form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__create_overlay_URL, body)
    return __get_resp_ovid(resp)
}

func Set_overlay(chinfo Channel) bool {
    v := url.Values{}

    if chinfo.Status == Preview {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RtspId))
    } else if chinfo.Status == Recording {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RecordingId))
    }
    v.Add("ov_id", chinfo.OvId)

    log.Printf("Preview set overlay form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__set_overlay_URL, body)
    return Get_resp_ret(resp)
}

func __remove_overlay(chinfo Channel) bool {
    v := url.Values{}

    if chinfo.Status == Preview {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RtspId))
    } else if chinfo.Status == Recording {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RecordingId))
    }
    v.Add("ov_id", chinfo.OvId)

    log.Printf("Preview remove overlay form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__remove_overlay_URL, body)
    return Get_resp_ret(resp)
}

func __delete_overlay(chinfo Channel) bool {
    v := url.Values{}

    if chinfo.Status == Preview {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RtspId))
    } else if chinfo.Status == Recording {
        v.Set("track_id", strconv.Itoa(chinfo.TrackIds.RecordingId))
    }
    v.Add("ov_id", chinfo.OvId)

    log.Printf("Preview delete overlay form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__delete_overlay_URL, body)
    return Get_resp_ret(resp)
}

func __overlay_on(chinfo Channel, ch int) bool {
    ovid, ret := __create_overlay(chinfo)

    if !ret {
        return false
    }

    chinfo.OvId = ovid

    CMap.ChMap_Set(ChList[ch], chinfo)

    if !Set_overlay(chinfo) {
        return false
    }
    return true
}

func __overlay_off(chinfo Channel) bool {
    if !__remove_overlay(chinfo) {
        return false
    }

    if !__delete_overlay(chinfo) {
        return false
    }
    return true
}

func __update_session_overlay_st(sess string, status bool) {
    sessinfo, _ := SMap.Get(sess)
    sessinfo.OvStatus = status
    SMap.Set(sess, sessinfo)
}

func __switch_overlay_on(w http.ResponseWriter, r *http.Request, chinfo Channel,
    ch int, sess string) {
    // check pCounter
    if chinfo.OverlayStatus {
        Resp_ret_reason(w, "Overlay has been opened")
        return
    }

    sessinfo, _ := SMap.Get(sess)

    // check status
    if !sessinfo.PreStatus && !sessinfo.RecStatus {
        Resp_ret_reason(w, "Please open preview or rec")
        return
    }

    if !__overlay_on(chinfo, ch) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    info, _ := CMap.ChMap_Get(ChList[ch])

    info.OverlayStatus = true

    CMap.ChMap_Set(ChList[ch], info)

    __update_session_overlay_st(sess, true)

    // resp ok
    Resp_ret(w, true)
}

func Switch_overlay_off(chinfo Channel, ch int, sess string) {
    // check pCounter
    if chinfo.PCounter > 1 {
        return
    }

    if !__overlay_off(chinfo) {
        return
    }

    chinfo.OverlayStatus = false

    CMap.ChMap_Set(ChList[ch], chinfo)

    __update_session_overlay_st(sess, false)
}

func __switch_overlay_off(w http.ResponseWriter, r *http.Request, chinfo Channel,
    ch int, sess string) {

    sessinfo, _ := SMap.Get(sess)
    // check session status
    if !sessinfo.OvStatus {
        Resp_ret(w, false)
        return
    }

    if !__overlay_off(chinfo) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    chinfo.OverlayStatus = false

    CMap.ChMap_Set(ChList[ch], chinfo)

    __update_session_overlay_st(sess, false)

    // resp ok
    Resp_ret(w, true)
}

func __overlay_post(w http.ResponseWriter, r *http.Request, chinfo Channel,
    ch int, sess string) {
    st, _ := Get_switch_status(r)

    if st {
        __switch_overlay_on(w, r, chinfo, ch, sess)
    } else {
        __switch_overlay_off(w, r, chinfo, ch, sess)
    }
}

func __overlay_get(w http.ResponseWriter, sessinfo Session) {
    m := &Resp_Switch{
        SwitchStatus: sessinfo.OvStatus,
    }
    __resp_overlay(w, m)
}

func OverlayHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    if !ok {
        log.Printf("Overlay error sess not exist \n")
        Resp_ret(w, false)
        return
    }

    // get chinfo
    ch := sessinfo.Channel
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    if r.Method == "GET" {
        __overlay_get(w, sessinfo)
    } else if r.Method == "POST" {
        __overlay_post(w, r, chinfo, ch, sess)
    }
}
