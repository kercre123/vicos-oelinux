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

package channel

import "C"
import "encoding/json"
import "net/http"
import "log"
import . "system/smap"
import "io/ioutil"
import "github.com/bitly/go-simplejson"
import . "common"

// resp json
type RespChannel struct {
    ChannelSelectVal int      `json:"channelSelectVal"`
    Channel          []string `json:"channel"`
    Status           bool     `json:"status"`
}

func __resp_channel(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    w.Write(j)
}

func __channel_get(w http.ResponseWriter, sessinfo Session) {
    // resp channel json
    m := &RespChannel{
        ChannelSelectVal: sessinfo.Channel,
        Channel:          ChList[0:MaxChannelNum],
        Status:           true,
    }

    __resp_channel(w, m)
}

func __get_switch_channel(body []byte) int {
    js, _ := simplejson.NewJson(body)

    ch, _ := js.Get("channelSelectVal").Int()

    log.Printf("Channel switch:%v \n", ch)

    return ch
}

func __channel_post(w http.ResponseWriter, r *http.Request, sess string, sessinfo Session) {
    // get switch channel
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    if len(body) == 0 {
        log.Printf("Error channel is null\n")
        Resp_ret(w, false)
        return
    }

    sessinfo.Channel = __get_switch_channel(body)

    SMap.Set(sess, sessinfo)
    // resp true
    Resp_ret(w, true)
}

func ChannelHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }

    sessinfo, ok := SMap.Get(sess)

    if !ok {
        log.Printf("Channel error sess not exist \n")
        Resp_ret(w, false)
        return
    }

    if r.Method == "GET" {
        __channel_get(w, sessinfo)
    } else if r.Method == "POST" {
        __channel_post(w, r, sess, sessinfo)
    }
}
