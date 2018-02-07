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

package recording

import (
    . "camera/media"
    . "common"
    "encoding/json"
    "log"
    "net/http"
    . "system/smap"
)

var __recording_num int = 0

type VideoIdParams struct {
    Video_Id int `json:"video_id"`
}

func __save_channel_state(chinfo Channel, ch int, status State) {
    log.Printf("Preview save channel state:%v \n", status)
    chinfo.Status = status
    CMap.ChMap_Set(ChList[ch], chinfo)
}

func __update_session_recording_st(sess string, status bool) {
    sessinfo, _ := SMap.Get(sess)
    sessinfo.RecStatus = status
    SMap.Set(sess, sessinfo)
}

func __check_camera_res(chinfo Channel) bool {
    // check preview num
    if __recording_num >= MaxRecNum {
        return false
    }

    // check res
    if (CameraRes - ResolutionConf[chinfo.VideoConf.Resolution]) < 0 {
        return false
    }
    return true
}

func __deal_camera_res(chinfo Channel, status State) {
    if status == Recording {
        CameraRes -= ResolutionConf[chinfo.VideoConf.Resolution]
    } else if status == Normal {
        CameraRes += ResolutionConf[chinfo.VideoConf.Resolution]
    }
}

func __save_video_id() {
    m := &VideoIdParams{
        Video_Id: VideoId,
    }

    j, _ := json.Marshal(m)

    Save_file(VIDEO_ID_FILE, j)
}

func __recording_on(chinfo Channel) bool {
    if !Create_video_track(chinfo, chinfo.TrackIds.RecordingId,
        chinfo.Sids.RecordingId, 2) {
        return false
    }

    if !Create_audio_track(chinfo.TrackIds.AudioId,
        chinfo.Sids.RecordingId, AudioConf.Track_output) {
        return false
    }

    if !Start_session(chinfo.Sids.RecordingId) {
        return false
    }

    return true
}

func __recording_off(chinfo Channel) bool {
    if !Stop_session(chinfo.Sids.RecordingId) {
        return false
    }
    if !Delete_video_track(chinfo.TrackIds.RecordingId,
        chinfo.Sids.RecordingId) {
        return false
    }

    if !Delete_audio_track(chinfo.TrackIds.AudioId,
        chinfo.Sids.RecordingId) {
        return false
    }

    Scan_media_file()
    return true
}

func __switch_recording_on(w http.ResponseWriter, chinfo Channel, sess string, ch int) {
    chState := chinfo.Status

    // check camera res
    if !__check_camera_res(chinfo) {
        log.Printf("Error need camrea res \n")
        Resp_ret_reason(w, "Resource limits")
        return
    }

    // check state
    if chState == Recording {
        log.Printf("Error other session recording \n")
        Resp_ret_reason(w, "The other session is recording")
        return
    }

    // preview on
    if chState == Preview {
        log.Printf("Error other session preview \n")
        Resp_ret(w, false)
        return
    }

    if !__recording_on(chinfo) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    // save map
    __save_channel_state(chinfo, ch, Recording)
    // deal camera res
    __deal_camera_res(chinfo, Recording)
    // add recording num
    __recording_num++
    // update session status
    __update_session_recording_st(sess, true)
    // resp ok
    Resp_ret(w, true)

}

func Switch_recording_off(chinfo Channel, sess string, ch int) {
    chState := chinfo.Status

    // check camera status
    if chState == Preview {
        return
    }

    // real preview off
    if !__recording_off(chinfo) {
        return
    }

    __save_channel_state(chinfo, ch, Normal)

    __deal_camera_res(chinfo, Normal)
    // delete preview num
    __recording_num--
    // update session status
    __update_session_recording_st(sess, false)
}

func __switch_recording_off(w http.ResponseWriter, chinfo Channel,
    sess string, ch int) {
    chState := chinfo.Status

    // check camera status
    if chState == Preview {
        Resp_ret(w, false)
        return
    }

    sessinfo, _ := SMap.Get(sess)

    // check session status
    if !sessinfo.RecStatus {
        Resp_ret(w, true)
        return
    }

    // real preview off
    if !__recording_off(chinfo) {
        Resp_ret(w, false)
        return
    }

    __save_channel_state(chinfo, ch, Normal)

    __deal_camera_res(chinfo, Normal)
    // delete preview num
    __recording_num--
    // update session status
    __update_session_recording_st(sess, false)

    Resp_ret(w, true)
}

func __recording_post(w http.ResponseWriter, r *http.Request, chinfo Channel,
    sess string, ch int) {
    st, _ := Get_switch_status(r)

    if st {
        __switch_recording_on(w, chinfo, sess, ch)
    } else {
        __switch_recording_off(w, chinfo, sess, ch)
    }
}

func RecordingHandler(w http.ResponseWriter, r *http.Request) {
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
        __recording_post(w, r, chinfo, sess, ch)
    } else if r.Method == "GET" {
        // check session status
        if sessinfo.RecStatus {
            Resp_ret(w, true)
        } else {
            Resp_ret(w, false)
        }
    }
}
