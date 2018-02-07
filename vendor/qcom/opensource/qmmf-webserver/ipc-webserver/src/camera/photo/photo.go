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

package photo

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

type ImageIdParams struct {
    Image_Id int `json:"image_id"`
}

const (
    __captrue_image_URL string = "http://127.0.0.1:4000/captureimage"
)

func __save_image_id() {
    m := &ImageIdParams{
        Image_Id: ImageId,
    }

    j, _ := json.Marshal(m)

    Save_file(IMAGE_ID_FILE, j)
}

func __resp_capture_image(w http.ResponseWriter, resp *http.Response) {

    body, err := ioutil.ReadAll(resp.Body)

    if err != nil {
        log.Printf("Server read body err:%v \n", err)
    }

    w.Header().Set("Content-Type", "application/json")
    w.Write(body)
}

func __capture_image(w http.ResponseWriter, chinfo Channel) {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))
    v.Set("image_width", strconv.Itoa(ResolutionOption[chinfo.VideoConf.Resolution].Width))
    v.Set("image_height", strconv.Itoa(ResolutionOption[chinfo.VideoConf.Resolution].Height))
    v.Set("image_quality", "100")

    log.Printf("Photo captureimage form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__captrue_image_URL, body)

    __resp_capture_image(w, resp)
}

func PhotoHandler(w http.ResponseWriter, r *http.Request) {
    sess, ok := CheckSession(r)
    if !ok {
        Resp_ret(w, false)
        return
    }
    sessinfo, ok := SMap.Get(sess)

    // get session channel
    ch := sessinfo.Channel
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    if r.Method == "POST" {

        __capture_image(w, chinfo)
    }
}
