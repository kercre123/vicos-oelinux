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

package preview

import "net/http"
import "log"
import . "system/smap"
import . "common"

var __preview_num int = 0

func __check_camera_res(chinfo Channel) bool {
    // check preview num
    if __preview_num >= MaxRtspNum {
        return false
    }
    log.Printf("__check_camera_res value: %v \n", ResolutionConf[chinfo.VideoConf.Resolution])
    log.Printf("__check_camera_res res: %v \n", CameraRes)
    // check res
    if (CameraRes - ResolutionConf[chinfo.VideoConf.Resolution]) < 0 {
        return false
    }
    return true
}

func __deal_camera_res(chinfo Channel, status State) {
    if status == Preview {
        CameraRes -= ResolutionConf[chinfo.VideoConf.Resolution]
    } else if status == Normal {
        CameraRes += ResolutionConf[chinfo.VideoConf.Resolution]
    }
}

func __preview_on(chinfo Channel, stream int) bool {
    log.Printf("Preview on stream:%v \n", stream)

    if stream == -1 || stream == Es {
        // only video 0 -> RTSP
        if !Create_video_track(chinfo, chinfo.TrackIds.RtspId,
        chinfo.Sids.PreviewId, 0) {
            return false
        }
    } else {
        // MpegTs 4 -> MpegTs muxing and streaming over RTSP
        if !Create_video_track(chinfo, chinfo.TrackIds.RtspId,
        chinfo.Sids.PreviewId, 4) {
            return false
        }

        if !Create_audio_track(chinfo.TrackIds.AudioId,
        chinfo.Sids.PreviewId, 2) {
            return false
        }
    }

    if !Start_session(chinfo.Sids.PreviewId) {
        return false
    }
    return true
}

func __preview_off(chinfo Channel) bool {
    log.Printf("Preview off stream:%v \n", chinfo.Stream)

    if !Stop_session(chinfo.Sids.PreviewId) {
        return false
    }

    if !Delete_video_track(chinfo.TrackIds.RtspId,
        chinfo.Sids.PreviewId) {
        return false
    }

    // MpegTs
    if chinfo.Stream == Ts {
        if !Delete_audio_track(chinfo.TrackIds.AudioId,
        chinfo.Sids.PreviewId) {
            return false
        }
    }
    return true
}

func __save_channel_state(chinfo Channel, ch int, status State) {
    log.Printf("Preview save channel state:%v \n", status)
    chinfo.Status = status
    CMap.ChMap_Set(ChList[ch], chinfo)
}

func __switch_preview_on(w http.ResponseWriter, chinfo Channel,
    sess string, ch int, stream int) {
    chState := chinfo.Status

    log.Printf("__switch_preview_on chinfo chstate: %v \n", chState)
    log.Printf("__switch_preview_on chinfo PCounter: %v \n", chinfo.PCounter)
    log.Printf("__switch_preview_on MaxRtspSessNum:%v \n", MaxRtspSessNum)

    // check pCounter
    if chinfo.PCounter >= MaxRtspSessNum {
        Resp_ret_reason(w, "Preview is already max")
        return
    }

    // preview on
    if chState == Preview {
        if chinfo.Stream != stream {
            Resp_ret_reason(w, "Please switch to another channel")
            return
        }
        // update session status
        __update_session_preview_st(sess, true)
        // preview counter +1
        chinfo.PCounter++
        CMap.ChMap_Set(ChList[ch], chinfo)
        // resp ok
        Resp_ret(w, true)
        return
    }

    // check camera res
    if !__check_camera_res(chinfo) {
        Resp_ret_reason(w, "Resource limits")
        return
    }

    // check state
    if chState == Recording {
        Resp_ret_reason(w, "Recording can not be interrupted")
        return
    }

    if !__preview_on(chinfo, stream) {
        Resp_ret_reason(w, "Internal error")
        return
    }

    // preview counter +1
    chinfo.PCounter++
    chinfo.Stream = stream
    CMap.ChMap_Set(ChList[ch], chinfo)
    // save map
    __save_channel_state(chinfo, ch, Preview)
    // deal camera res
    __deal_camera_res(chinfo, Preview)
    // add preview num
    __preview_num++
    // update session status
    __update_session_preview_st(sess, true)
    // resp ok
    Resp_ret(w, true)
}

func Switch_preview_off(chinfo Channel, sess string, ch int) {
    chState := chinfo.Status

    log.Printf("Preview previewoff PCounter:%v \n", chinfo.PCounter)
    // preview off
    if chState == Preview && chinfo.PCounter != 1 {
        // not real off
        __update_session_preview_st(sess, false)
        // counter --
        chinfo.PCounter--
        CMap.ChMap_Set(ChList[ch], chinfo)

        Close_rtsp_proxy(RTSP, sess)

        return
    }

    // check camera status
    if chState == Recording {
        return
    }

    // real preview off
    if !__preview_off(chinfo) {
        return
    }
    // preview counter -1
    chinfo.PCounter--
    chinfo.Stream = -1

    __save_channel_state(chinfo, ch, Normal)

    __deal_camera_res(chinfo, Normal)
    // delete preview num
    __preview_num--
    // update session status
    __update_session_preview_st(sess, false)

    Close_rtsp_proxy(RTSP, sess)
}

func __switch_preview_off(w http.ResponseWriter, chinfo Channel,
    sess string, ch int) {

    sessinfo, _ := SMap.Get(sess)

    chState := chinfo.Status

    log.Printf("Preview previewoff PCounter:%v \n", chinfo.PCounter)
    // preview off
    if chState == Preview && chinfo.PCounter != 1 {
        // not real off
        __update_session_preview_st(sess, false)
        // counter --
        chinfo.PCounter--
        CMap.ChMap_Set(ChList[ch], chinfo)

        Close_rtsp_proxy(RTSP, sess)
        // resp ok
        Resp_ret(w, true)
        return
    }

    // check camera status
    if chState == Recording {
        Resp_ret(w, false)
        return
    }

    // check session status
    if !sessinfo.PreStatus {
        Resp_ret(w, true)
        return
    }

    // real preview off
    if !__preview_off(chinfo) {
        Resp_ret_reason(w, "Internal error")
        return
    }
    // preview counter -1
    chinfo.PCounter--
    chinfo.Stream = -1

    __save_channel_state(chinfo, ch, Normal)

    __deal_camera_res(chinfo, Normal)
    // delete preview num
    __preview_num--
    // update session status
    __update_session_preview_st(sess, false)

    Close_rtsp_proxy(RTSP, sess)

    //resp ok
    Resp_ret(w, true)
}

func __preview_post(w http.ResponseWriter, r *http.Request, chinfo Channel,
    sess string, ch int) {
    st, stream := Get_stream_mode(r)

    if st {
        __switch_preview_on(w, chinfo, sess, ch, stream)
    } else {
        __switch_preview_off(w, chinfo, sess, ch)
    }
}

func __preview_get(w http.ResponseWriter, chinfo Channel, sess string, sessinfo Session) {
    chState := chinfo.Status

    log.Printf("Preview get channel status:%v \n", chState)
    if chState != Preview {
        Resp_ret(w, false)
        return
    }

    rtsp_url := Get_server_info(chinfo.Sids.PreviewId)
    log.Printf("Preview get rtsp url:%v \n", rtsp_url)

    if rtsp_url == "" {
        log.Printf("Get rtsp url is nil!!!")
        Resp_ret_reason(w, "Internal error")
        return
    }

    if sessinfo.ProxyConf[RTSP].Port == "" {
        port := Set_rtsp_proxy(RTSP, sess, sessinfo)
        log.Printf("Preview get proxy port:%v \n", port)
    }

    rtspURL := rtsp_url
    if IsWifiAPMode {
        rtspURL = Make_resp_rtspURL(CameraIp, rtsp_url)
    }
    log.Printf("Preview resp rtsp url:%v \n", rtspURL)

    // get new sessinfo
    sessinfo, _ = SMap.Get(sess)

    m := &RespRtspURL{
        Url:    rtspURL,
        Proxy:  sessinfo.ProxyConf[RTSP].Port,
        Status: true,
    }

    Resp_url(w, m)
}

func __update_session_preview_st(sess string, status bool) {
    sessinfo, _ := SMap.Get(sess)
    sessinfo.PreStatus = status
    // clean rtsp port
    if !status {
        sessinfo.ProxyConf[RTSP].Port = ""
    }
    SMap.Set(sess, sessinfo)
}

func PreviewHandler(w http.ResponseWriter, r *http.Request) {
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
        __preview_post(w, r, chinfo, sess, ch)
    } else if r.Method == "GET" {
        // check session status
        if !sessinfo.PreStatus {
            Resp_ret(w, false)
            return
        }
        __preview_get(w, chinfo, sess, sessinfo)
    }
}
