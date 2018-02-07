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

package server

/*
#cgo LDFLAGS: -lcutils

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <cutils/properties.h>
#include <unistd.h>

volatile uint32_t camera_mode_;

uint32_t camera_mode() {
  char prop[PROPERTY_VALUE_MAX];
  property_get("persist.qmmf.ipcweb.camtype", prop, 0);
  camera_mode_ = atoi (prop);
  return camera_mode_;
}

int32_t get_wifi_mode(void) {
    char prop[2];
    property_get("persist.qmmf.ws.wifi.ap.mode", prop, 0);
    return atoi(prop);
}
*/
import "C"
import (
    "bytes"
    . "camera/media"
    . "common"
    "github.com/bitly/go-simplejson"
    "strconv"
    "encoding/json"
    "io/ioutil"
    "log"
    "net/url"
    "net/http"
    . "network"
    . "os/exec"
    "strings"
    . "system/smap"
    "os"
)

const (
    CAMERA_IP = iota
    CAMERA_360
)

const (
    RES_CONF string = Webserver_dir + "/res_config"

    camIP_conf string = "/ip_cam.conf"
    cam360_conf string = "/360_cam.conf"

    // qmmfwebserver url
    __connect_URL            string = "http://127.0.0.1:4000/connect"
    __start_camera_URL       string = "http://127.0.0.1:4000/startcamera"
    __disconnect_URL         string = "http://127.0.0.1:4000/disconnect"
    __stop_camera_URL        string = "http://127.0.0.1:4000/stopcamera"
    __delete_session_URL     string = "http://127.0.0.1:4000/deletesession"
    __create_multicamera_URL string = "http://127.0.0.1:4000/createmulticamera"

    // cmd
    __GET_SSID_CMD   string = "cat /data/misc/wifi/hostapd.conf | grep \"^ssid=\" | sed 's/^.*ssid=//g'"
    __GET_PASSWD_CMD string = "cat /data/misc/wifi/hostapd.conf | grep \"^wpa_passphrase=\" | sed 's/^.*wpa_passphrase=//g'"
)

func __connect() bool {
    respCnn := Http_post(__connect_URL, nil)

    return Get_resp_ret(respCnn)
}


func __disconnect() bool {
    respCnn := Http_post(__disconnect_URL, nil)

    return Get_resp_ret(respCnn)
}

func __start_camera() bool {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))
    v.Add("zsl_mode", "0")
    v.Add("zsl_width", "1920")
    v.Add("zsl_height", "1080")
    v.Add("zsl_queue_depth", "8")
    v.Add("framerate", "30")
    v.Add("flags", "0")

    log.Printf("Server start camera HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    respSC := Http_post(__start_camera_URL, body)
    return Get_resp_ret(respSC)
}

func __stop_camera() bool {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))

    log.Printf("Server stop camera HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    respSC := Http_post(__stop_camera_URL, body)
    return Get_resp_ret(respSC)
}

func __delete_session(sessid int) bool {
    v := url.Values{}
    v.Add("session_id", strconv.Itoa(sessid))

    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__delete_session_URL, body)
    return Get_resp_ret(resp)
}

func __create_multicamera() int {
    v := url.Values{}
    v.Set("camera_ids", "[0,1]")

    log.Printf("Server create multicamera HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    respSC := Http_post(__create_multicamera_URL, body)
    vcid := __get_resp_vcid(respSC)
    return vcid
}

func __get_resp_vcid(resp *http.Response) int {
    body, err := ioutil.ReadAll(resp.Body)

    if err != nil {
        log.Printf("Server read body err:%v \n", err)
    }
    log.Printf("Server connect resp body:%v \n", string(body))

    js, err := simplejson.NewJson(body)

    sid, _ := js.Get("VirtualCameraId").Int()

    defer resp.Body.Close()

    return sid
}

func __parse_device_config() {
    var file_name string
    if (Is360Cam()) {
        file_name = Webserver_dir + cam360_conf
    } else {
        file_name = Webserver_dir + camIP_conf
    }

    if Exist(file_name) {
        body, err := ioutil.ReadFile(file_name)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error device config is null\n")
            return
        }

        _ = json.Unmarshal([]byte(body),&DeviceConfig)

        log.Printf("Camera Type %v\n",DeviceConfig.CamType)

        for _, v := range DeviceConfig.CamResolutionRange {
            log.Printf("OK %v \n",v)

            stringSlice := strings.Split(v, "x")
            w, _ := strconv.Atoi(stringSlice[0])
            h, _ := strconv.Atoi(stringSlice[1])

            AddResolutionConf(w*h)
            AddResolutionOption(w,h)
            AddResolutionList(v)
        }
            stringSlice := strings.Split(DeviceConfig.VAMResolutionVal, "x")
            w, _ := strconv.Atoi(stringSlice[0])
            h, _ := strconv.Atoi(stringSlice[1])
            SetVAMResolution(w,h)
    }
}

func __sync_camera_config() {
    if Exist(TMP_CAMERA_IP_FILE) {
        IsRecovery = true
        ip,_ := ioutil.ReadFile(TMP_CAMERA_IP_FILE)
        // sync cameraip
        CameraIp = string(ip)
        // delete tmp cameraip file
        os.Remove(TMP_CAMERA_IP_FILE)
        // Insert media list
        Scan_media_file()
    }
}

func __set_video_default_conf(index int, preview_sid int, recording_sid int, vam_sid int) {
    CMap.ChMap_Set(ChConfList[index].Ch, Channel{index, Sess_id{preview_sid, recording_sid, vam_sid},
        0, 0, [5]VamConf{{0, false, ""}, {0, false, ""}, {0, false, ""}, {0, false, ""}, {0, false,""}}, Normal, VConf{1, 0, 6, 1}, // 1080P 4Mbps
        Track_Id{index + 1, index + 3, index + 5, index + 7},
        "", false, 0, [3]OvConf{{0, "869007615", 0, "Snapdragon 625 IPCamera", 0, 0, 0, 0, 0, "Snapdragon 625 IPCamera", 0, 0},
            {0, "869007615", 0, "Snapdragon 625 IPCamera", 0, 0, 0, 0, 0, "Snapdragon 625 IPCamera", 0, 0},
            {0, "869007615", 0, "Snapdragon 625 IPCamera", 0, 0, 0, 0, 0, "Snapdragon 625 IPCamera", 0, 0}}, -1})
}

func __sync_channel_config() {
    var index int

    for index = 0; index < MaxChannelNum; index++ {
        // create session
        preview_sid := Create_session()
        recording_sid := Create_session()
        vam_sid := Create_session()

        if Exist(ChConfList[index].Video_Conf) {
            body, err := ioutil.ReadFile(ChConfList[index].Video_Conf)

            if err != nil {
                panic(err)
            }

            log.Printf("video config length is :%v\n", len(body))

            if len(body) == 0 {
                log.Printf("Error video config is null\n")
                // default 4K HEVC
                __set_video_default_conf(index, preview_sid, recording_sid, vam_sid)
                return
            }

            js, err := simplejson.NewJson(body)

            resolution, _ := js.Get("resolutionSelectVal").Int()
            encode, _ := js.Get("encodeModeSelectVal").Int()
            bitrate, _ := js.Get("bitRateSelectVal").Int()
            fps, _ := js.Get("fpsSelectVal").Int()

            CMap.ChMap_Set(ChConfList[index].Ch, Channel{index, Sess_id{preview_sid, recording_sid,
                vam_sid}, 0, 0, [5]VamConf{{0, false, ""}, {0, false, ""}, {0, false, ""}, {0, false, ""}, {0, false,""}},
                Normal, VConf{resolution, encode, bitrate, fps},
                Track_Id{index + 1, index + 3, index + 5, index + 7},
                "", false, 0, [3]OvConf{{0, "869007615", 0, "Snapdragon 625 IPCamera", 0, 0, 0, 0, 0, "Snapdragon 625 IPCamera", 0, 0},
                    {0, "869007615", 0, "Snapdragon 625 IPCamera", 0, 0, 0, 0, 0, "Snapdragon 625 IPCamera", 0, 0},
                    {0, "869007615", 0, "Snapdragon 625 IPCamera", 0, 0, 0, 0, 0, "Snapdragon 625 IPCamera", 0, 0}}, -1})
        } else {
            // default 4K HEVC
            __set_video_default_conf(index, preview_sid, recording_sid, vam_sid)
        }
    }
}

func __destroy_channels() {
    var index int

    for index = 0; index < MaxChannelNum; index++ {
        // destroy channel and session

        ch1, _ := CMap.GetRef(ChList[index])
        preview_sid   := ch1.Sids.PreviewId
        recording_sid := ch1.Sids.RecordingId
        vam_sid       := ch1.Sids.VamId

        // delete session
        __delete_session(preview_sid)
        __delete_session(recording_sid)
        __delete_session(vam_sid)

        // delete channel info
        CMap.ChMap_Remove(ChList[index])
    }
}

func __sync_overlay_conifg() {
    for index := 0; index < MaxChannelNum; index++ {
        Sync_channel_overlay_conf(index, ChConfList[index].Ch)
    }
}

func __sync_res_config() {
    if Exist(RES_CONF) {
        body, err := ioutil.ReadFile(RES_CONF)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error res config is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        MaxSessNum, _ = js.Get("max_session").Int()
        MaxChannelNum, _ = js.Get("max_channel").Int()
        MaxRtspNum, _ = js.Get("max_rtsp").Int()
        MaxRtspSessNum, _ = js.Get("max_rtsp_sess").Int()
        MaxRecNum, _ = js.Get("max_recording").Int()
        CameraRes, _ = js.Get("camera_res").Int()
    }
}

func __sync_network_config() {
    if Exist(NET_CONF) {
        body, err := ioutil.ReadFile(NET_CONF)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error network config is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        NetConf.Mode, _ = js.Get("modeSelectVal").Int()

        log.Printf("NetWork mode: %v", NetConf.Mode)
        log.Printf("Recoverymode is %v", IsRecovery)

        if NetConf.Mode == 0 {
            NetConf.Ip = ""
            NetConf.Mask = ""
            // exec dhcp cmd
            if !IsRecovery {
                Deal_dhcp_cmd()
            }
            return
        }

        if NetConf.Mode == 1 {
            NetConf.Ip, _ = js.Get("ip").String()
            NetConf.Mask, _ = js.Get("mask").String()
            NetConf.RangeStart, _ = js.Get("range_start").String()
            NetConf.RangeEnd, _ = js.Get("range_end").String()
            // exec ip config cmd
            if !IsRecovery {
                Deal_static_cmd(body)
            }
            return
        }
    } else {
        // default dhcp
        log.Printf("Recoverymode is %v", IsRecovery)
        if !IsRecovery {
            Deal_dhcp_cmd()
        }
    }
}

func __exec_get_wifi_cmd(getcmd string) string {
    cmd := Command("/bin/sh", "-c", getcmd)

    var out bytes.Buffer
    cmd.Stdout = &out

    err := cmd.Run()

    if err != nil {
        log.Printf("Error exec getssid cmd: %v", err)
    }

    return strings.Replace(out.String(), "\n", "", -1)
}

func __sync_wifi_config() {
    if Exist(WIFI_CONF) {
        body, err := ioutil.ReadFile(WIFI_CONF)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error wifi config is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        mode, _ := js.Get("modeSelectVal").Int()

        log.Printf("Wifi mode: %v", mode)

        if mode == 0 {
            ApConf.Mode = mode
            ApConf.Ssid, _ = js.Get("ssid").String()
            ApConf.Password, _ = js.Get("password").String()

            log.Printf("Hostapd SSID: %v", ApConf.Ssid)
            log.Printf("Hostapd Password: %v", ApConf.Password)
            return
        }
    } else {
        // default
        if Exist(HOSTAPD_CONF) {
            ApConf.Mode = 0
            ApConf.Ssid = __exec_get_wifi_cmd(__GET_SSID_CMD)
            ApConf.Password = __exec_get_wifi_cmd(__GET_PASSWD_CMD)

            log.Printf("Default Hostapd SSID: %v", ApConf.Ssid)
            log.Printf("Default Hostapd Password: %v", ApConf.Password)
            return
        }
    }

}

func __sync_media_list() {
    Scan_media_file()
}

func __sync_media_conf() {
    if Exist(VIDEO_CONF) {
        body, err := ioutil.ReadFile(VIDEO_CONF)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error video config is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        VPNum, _ = js.Get("perPage").Int()
    }

    if Exist(IMAGE_CONF) {
        body, err := ioutil.ReadFile(IMAGE_CONF)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error image config is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        IPNum, _ = js.Get("perPage").Int()
    }
}

func __sync_enroll_id() {
    if Exist(ENROLL_ID_FILE) {
        body, err := ioutil.ReadFile(ENROLL_ID_FILE)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error enroll id file is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        EnrollId, _ = js.Get("enroll_id").Int()

        log.Printf("EnrollId is %v\n", EnrollId)
    }
}

func __sync_image_id() {
    if Exist(IMAGE_ID_FILE) {
        body, err := ioutil.ReadFile(IMAGE_ID_FILE)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error image id file is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        ImageId, _ = js.Get("image_id").Int()

        log.Printf("image id is %v\n", ImageId)
    }
}

func __sync_video_id() {
    if Exist(VIDEO_ID_FILE) {
        body, err := ioutil.ReadFile(VIDEO_ID_FILE)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error video id file is null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        VideoId, _ = js.Get("video_id").Int()

        log.Printf("video id is %v\n", VideoId)
    }
}

func __sync_onvif() {
    if Exist(ONVIF_FLAG_FILE) {
        OnvifSt = true
    }
}

func __sync_audio_config() {
    if Exist(AUDIO_CONFIG_FILE) {
        body, err := ioutil.ReadFile(AUDIO_CONFIG_FILE)

        if err != nil {
            panic(err)
        }

        if len(body) == 0 {
            log.Printf("Error audio config file null\n")
            return
        }

        js, err := simplejson.NewJson(body)

        AudioConf.Sample_rate, _ = js.Get("sample_rate").Int()
        AudioConf.Num_channels, _ = js.Get("num_channels").Int()
        AudioConf.Bit_depth, _ = js.Get("bit_depth").Int()
        AudioConf.Track_codec, _ = js.Get("track_codec").Int()
        AudioConf.Track_output, _ = js.Get("track_output").Int()
        AudioConf.Bitrate, _ = js.Get("bitrate").Int()
    }
}

func Is360Cam() bool {
    var cam_mode =  C.camera_mode()
    if cam_mode == CAMERA_IP {
        return false
    } else {
        return true
    }
}

func __initWifiAPMode() {
    IsWifiAPMode = (C.get_wifi_mode() == 1)
}

func InitCamera() {
    log.Printf("Server init\n")

    // Check wifi mode
    __initWifiAPMode()
    // sync res config
    __sync_res_config()
    // sync camera config
    __sync_camera_config()
    // sync net config
    __sync_network_config()
    // sync wifi config
    __sync_wifi_config()
    // sync media conf
    __sync_media_conf()
    // sync vam enroll id
    __sync_enroll_id()
    // sync image id
    __sync_image_id()
    // sync video id
    __sync_video_id()
    // sync onvif
    __sync_onvif()
    // sync audio config
    __sync_audio_config()
}

func InitConnection() {

    // connect
    if !__connect() {
        return
    }

    __parse_device_config()

    if Is360Cam() {
        var vid = __create_multicamera()
        SetCameraID(vid)
        log.Printf("Camera Running in Multi Camera mode\n")
    } else {
        log.Printf("Camera Running in Single Camera mode\n")
    }

    if !__start_camera() {
        return
    }

    // sync video config
    __sync_channel_config()
    // sync overlay config
    __sync_overlay_conifg()
    // sync net config
}

func DeInitConnection() {

    // destroy channel
    __destroy_channels()

    //stop camera
    if !__stop_camera() {
        return
    }

    // disconnect
    if !__disconnect() {
        return
    }
}
