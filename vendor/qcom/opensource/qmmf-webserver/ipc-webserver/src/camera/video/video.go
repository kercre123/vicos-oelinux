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

package video

import "encoding/json"
import "net/http"
import "log"
import . "system/smap"
import "io/ioutil"
import "github.com/bitly/go-simplejson"
import . "common"
import "os"

// resp json
type RespVideo struct {
    ResolutionSelectVal int      `json:"resolutionSelectVal"`
    Resolution          []string `json:"resolution"`
    EncodeModeSelectVal int      `json:"encodeModeSelectVal"`
    EncodeMode          []string `json:"encodeMode"`
    BitRateSelectVal    int      `json:"bitRateSelectVal"`
    BitRate             []string `json:"bitRate"`
    FpsSelectVal        int      `json:"fpsSelectVal"`
    Fps                 []int    `json:"fps"`
    Status              bool     `json:"status"`
}

func __save_channel_video_conf(conf []byte, ch int) {
    f, err := os.Create(ChConfList[ch].Video_Conf)

    if err != nil {
        panic("error")
    }
    defer f.Close()

    f.Write(conf)
    f.Sync()
}

func __resp_video(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    w.Write(j)
}

func __save_channel_video_map(chinfo Channel, body []byte, ch int) {

    if len(body) == 0 {
        log.Printf("Error save video is null\n")
        return
    }

    js, _ := simplejson.NewJson(body)

    chinfo.VideoConf.Resolution, _ = js.Get("resolutionSelectVal").Int()
    chinfo.VideoConf.Encode, _ = js.Get("encodeModeSelectVal").Int()
    chinfo.VideoConf.Bitrate, _ = js.Get("bitRateSelectVal").Int()
    chinfo.VideoConf.Fps, _ = js.Get("fpsSelectVal").Int()

    // set Cmap
    CMap.ChMap_Set(ChList[ch], chinfo)
}

func __video_get(w http.ResponseWriter, chinfo Channel) {
    // resp channel json
    m := &RespVideo{
        ResolutionSelectVal: chinfo.VideoConf.Resolution,
        Resolution:          ResolutionList,
        EncodeModeSelectVal: chinfo.VideoConf.Encode,
        EncodeMode:          EncodeList,
        BitRateSelectVal:    chinfo.VideoConf.Bitrate,
        BitRate:             BitRateShowList,
        FpsSelectVal:        chinfo.VideoConf.Fps,
        Fps:                 FpsList,
        Status:              true,
    }

    __resp_video(w, m)
}

func __video_post(w http.ResponseWriter, r *http.Request, chinfo Channel, ch int) {
    // channel status
    chState := chinfo.Status

    if chState != Normal {
        Resp_ret_reason(w, "The other session is preview or recording")
        return
    }

    body, err := ioutil.ReadAll(r.Body)
    if err != nil {
        panic("error")
    }
    // save config file
    __save_channel_video_conf(body, ch)
    // save config map
    __save_channel_video_map(chinfo, body, ch)

    Resp_ret(w, true)
}

func VideoHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    if !ok {
        log.Printf("Video error sess not exist \n")
        Resp_ret(w, false)
        return
    }

    // get chinfo
    ch := sessinfo.Channel
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    if r.Method == "GET" {
        __video_get(w, chinfo)
    } else if r.Method == "POST" {
        __video_post(w, r, chinfo, ch)
    }
}
