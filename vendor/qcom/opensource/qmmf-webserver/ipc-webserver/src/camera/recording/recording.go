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
    . "camera/preview"
    . "camera/overlay"
    "time"
)

var __recording_num int = 0

var is4KRecording bool = false

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

    log.Printf("recording __check_camera_res value: %v \n", ResolutionConf[chinfo.VideoConf.Resolution])
    log.Printf("recording __check_camera_res res: %v \n", CameraRes)
    // check res
    if (CameraRes - ResolutionConf[chinfo.VideoConf.Resolution]) < 0 {
        return false
    }
    return true
}

func __deal_camera_res(chinfo Channel, status State) {
    log.Printf("deal camera res Resolution: %v, state:%v \n", chinfo.VideoConf.Resolution, status)
    if status == Recording {
        CameraRes -= ResolutionConf[chinfo.VideoConf.Resolution]
    } else {
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

    if !Create_audio_track(chinfo.TrackIds.RecAudioId,
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

    if !Delete_audio_track(chinfo.TrackIds.RecAudioId,
        chinfo.Sids.RecordingId) {
        return false
    }

    Scan_media_file()
    return true
}

func __switch_recording_on(w http.ResponseWriter, chinfo Channel, sess string, ch int) {
    chState := chinfo.Status

    // check state
    if chState == Recording {
        log.Printf("Error other session recording \n")
        Resp_ret_reason(w, "The other session is recording")
        return
    }

    // check couter
    if chinfo.PCounter > 1 {
        log.Printf("Error other session preview \n")
        Resp_ret_reason(w, "The other session is preview")
        return
    }

    // if video = 4K
    if chinfo.VideoConf.Resolution == 0 {
        // wait web stop websocket
        time.Sleep(50 * time.Millisecond)

        // off current preview
        if !Preview_video_off(chinfo, sess, ch) {
            return
        }

        chinfo, _ = CMap.ChMap_Get(ChList[ch])
        // set video 480
        chinfo.VideoConf.Resolution = 3
        CMap.ChMap_Set(ChList[ch], chinfo)

        // on 480 preview
        if !Preview_video_on(sess, ch, chinfo.Stream) {
            return
        }

        if chinfo.OverlayStatus {
            if !Pre_overlay_on(ch) {
                return
            }
        }

        is4KRecording = true
    }

    // check camera res
    if !__check_camera_res(chinfo) {
        log.Printf("Error need camrea res \n")
        Resp_ret_reason(w, "Resource limits")
        return
    }

    // if video = 4K
    if chinfo.VideoConf.Resolution == 0 {
        // off current preview
        if !Preview_video_off(chinfo, sess, ch) {
            return
        }

        chinfo, _ = CMap.ChMap_Get(ChList[ch])
        // set video 480
        chinfo.VideoConf.Resolution = 3
        CMap.ChMap_Set(ChList[ch], chinfo)

        // on 480 preview
        if !Preview_video_on(sess, ch, chinfo.Stream) {
            return
        }

        if chinfo.OverlayStatus {
            if !Pre_overlay_on(ch) {
                return
            }
        }

        is4KRecording = true
    }

    // check camera res
    if !__check_camera_res(chinfo) {
        log.Printf("Error need camrea res \n")
        Resp_ret_reason(w, "Resource limits")
        return
    }

    if is4KRecording {
        // get new chinfo
        chinfo, _ = CMap.ChMap_Get(ChList[ch])

        // revert 4K video
        chinfo.VideoConf.Resolution = 0
        CMap.ChMap_Set(ChList[ch], chinfo)
    }

    if !__recording_on(chinfo) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    // open overlay
    if chinfo.OverlayStatus {
        if !Rec_overlay_on(ch) {
            Resp_ret_reason(w, "Internal error")
            return
        }
    }

    // get new chinfo
    chinfo, _ = CMap.ChMap_Get(ChList[ch])

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

    // 4k close 480 preview
    if chinfo.VideoConf.Resolution == 0 {
        chinfo, _ = CMap.ChMap_Get(ChList[ch])

        // set video 480
        chinfo.VideoConf.Resolution = 3
        CMap.ChMap_Set(ChList[ch], chinfo)

        if !Preview_video_off(chinfo, sess, ch) {
            return
        }

        // set video 4K
        chinfo.VideoConf.Resolution = 0
        CMap.ChMap_Set(ChList[ch], chinfo)

        return
    }

    if !Preview_video_off(chinfo, sess, ch) {
        return
    }
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

    // close overlay
    if chinfo.OverlayStatus {
        if !Rec_overlay_off(chinfo) {
            Resp_ret_reason(w, "Internal error")
            return
        }
    }

    // real preview off
    if !__recording_off(chinfo) {
        Resp_ret(w, false)
        return
    }

    __save_channel_state(chinfo, ch, Preview)

    __deal_camera_res(chinfo, Preview)
    // delete preview num
    __recording_num--
    // update session status
    __update_session_recording_st(sess, false)

    if chinfo.VideoConf.Resolution == 0 {
        chinfo, _ = CMap.ChMap_Get(ChList[ch])

        // set video 480
        chinfo.VideoConf.Resolution = 3
        CMap.ChMap_Set(ChList[ch], chinfo)

        if chinfo.OverlayStatus {
            if !Pre_overlay_off(chinfo) {
                return
            }
        }

        // get new chinfo
        chinfo, _ = CMap.ChMap_Get(ChList[ch])

        if !Preview_video_off(chinfo, sess, ch) {
            return
        }

        chinfo, _ = CMap.ChMap_Get(ChList[ch])
        // set video 4K
        chinfo.VideoConf.Resolution = 0
        CMap.ChMap_Set(ChList[ch], chinfo)

        if !Preview_video_on(sess, ch, chinfo.Stream) {
            return
        }

        if chinfo.OverlayStatus {
            if !Pre_overlay_on(ch) {
                return
            }
        }

        is4KRecording = true
    }

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

func __recording_get(w http.ResponseWriter, chinfo Channel, sess string, sessinfo Session) {
    rtsp_url := Get_server_info(chinfo.Sids.PreviewId)
    log.Printf("Recording get resolution:%v \n", chinfo.VideoConf.Resolution)

    if chinfo.VideoConf.Resolution != 0 {
        Resp_ret(w, true)
        return
    }

    if rtsp_url == "" {
        log.Printf("Get rtsp url is nil!!!")
        Resp_ret_reason(w, "Internal error")
        return
    }

    if sessinfo.ProxyConf[RTSP].Port == "" {
        port := Set_rtsp_proxy(RTSP, sess, sessinfo)
        log.Printf("Recording get proxy port:%v \n", port)
    }

    rtspURL := Make_resp_rtspURL(CameraIp, rtsp_url)
    log.Printf("Recording resp rtsp url:%v \n", rtspURL)

    // get new sessinfo
    sessinfo, _ = SMap.Get(sess)

    m := &RespRtspURL{
        Url:    rtspURL,
        Proxy:  sessinfo.ProxyConf[RTSP].Port,
        Status: true,
    }

    is4KRecording = false

    Resp_url(w, m)
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
        if !is4KRecording {
            if !sessinfo.RecStatus {
                Resp_ret(w, false)
                return
            }
        }
        __recording_get(w, chinfo, sess, sessinfo)
    }
}
