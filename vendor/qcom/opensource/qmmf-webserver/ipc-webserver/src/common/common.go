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

package common

import (
    "encoding/json"
    "github.com/bitly/go-simplejson"
    "io"
    "io/ioutil"
    "log"
    "net/http"
    "net/url"
    "os"
    . "os/exec"
    "strconv"
    "strings"
    . "system/bitmap"
    . "system/smap"
)

const (
    RTSP = iota // value --> 0
    FR          // value --> 1
    CT          // value --> 2
    MD          // value --> 3
    OC
    MAX
)

const (
    // webserver dir
    Webserver_dir string = "/data/misc/qmmf/ipc_webserver"

    // server config
    CHANEEL_ONE_VIDEO_CONF   string = Webserver_dir + "/channel1/video_config"
    CHANEEL_TWO_VIDEO_CONF   string = Webserver_dir + "/channel2/video_config"
    CHANEEL_THREE_VIDEO_CONF string = Webserver_dir + "/channel3/video_config"

    CHANEEL_ONE_OVERLAY_CONF   string = Webserver_dir + "/channel1/ov_config"
    CHANEEL_TWO_OVERLAY_CONF   string = Webserver_dir + "/channel2/ov_config"
    CHANEEL_THREE_OVERLAY_CONF string = Webserver_dir + "/channel3/ov_config"

    CHANEEL_ONE_TEXT_OVERLAY_CONF   string = Webserver_dir + "/channel1/ov_text_config"
    CHANEEL_TWO_TEXT_OVERLAY_CONF   string = Webserver_dir + "/channel2/ov_text_config"
    CHANEEL_THREE_TEXT_OVERLAY_CONF string = Webserver_dir + "/channel3/ov_text_config"

    CHANEEL_ONE_TIME_OVERLAY_CONF   string = Webserver_dir + "/channel1/ov_time_config"
    CHANEEL_TWO_TIME_OVERLAY_CONF   string = Webserver_dir + "/channel2/ov_time_config"
    CHANEEL_THREE_TIME_OVERLAY_CONF string = Webserver_dir + "/channel3/ov_time_config"

    CHANEEL_ONE_BOX_OVERLAY_CONF   string = Webserver_dir + "/channel1/ov_box_config"
    CHANEEL_TWO_BOX_OVERLAY_CONF   string = Webserver_dir + "/channel2/ov_box_config"
    CHANEEL_THREE_BOX_OVERLAY_CONF string = Webserver_dir + "/channel3/ov_box_config"

    // net config
    NET_CONF     string = Webserver_dir + "/net_config"
    WIFI_CONF    string = Webserver_dir + "/wifi_config"
    HOSTAPD_CONF string = "/data/misc/wifi/hostapd.conf"

    // media config
    VIDEO_CONF string = Webserver_dir + "/video_config"
    IMAGE_CONF string = Webserver_dir + "/image_config"

    // media dir
    MEDIA_DIR string = "/data/misc/qmmf/"

    // vam enroll id
    ENROLL_ID_FILE = Webserver_dir + "/vam/enroll_id"

    // image id
    IMAGE_ID_FILE = Webserver_dir + "/image/image_id"

    // video id
    VIDEO_ID_FILE = Webserver_dir + "/video/video_id"

    // Onvif flag file
    ONVIF_FLAG_FILE string = Webserver_dir + "/onvif_mode.flag"

    // tmp video id file
    TMP_VIDEO_ID_FILE = Webserver_dir + "/video_id"

    // tmp image id file
    TMP_IMAGE_ID_FILE = Webserver_dir + "/image_id"

    // tmp cameraip file
    TMP_CAMERA_IP_FILE string = "/var/run/cameraip"

    // Audio config file
    AUDIO_CONFIG_FILE string = Webserver_dir + "/audio_config"

    // post common url
    __create_video_track_URL  string = "http://127.0.0.1:4000/createvideotrack"
    __start_session_URL      string = "http://127.0.0.1:4000/startsession"
    __delete_video_track_URL string = "http://127.0.0.1:4000/deletevideotrack"
    __stop_session_URL       string = "http://127.0.0.1:4000/stopsession"
    __create_session_URL     string = "http://127.0.0.1:4000/createsession"
    __create_audio_track_URL  string = "http://127.0.0.1:4000/createaudiotrack"
    __delete_audio_track_URL string = "http://127.0.0.1:4000/deleteaudiotrack"
    Get_server_info_URL      string = "http://127.0.0.1:4000/"
)

type ChConf struct {
    Ch           string
    Video_Conf   string
    Ov_Conf      string
    Ov_Text_Conf string
    Ov_Time_Conf string
    Ov_Box_Conf  string
}

// Camera list
var CamList = []string{"Camera One"}

// Channel list
var ChList = []string{"Channel One", "Channel Two", "Channel Three"}

var ChConfList = []ChConf{{"Channel One", CHANEEL_ONE_VIDEO_CONF, CHANEEL_ONE_OVERLAY_CONF,
    CHANEEL_ONE_TEXT_OVERLAY_CONF, CHANEEL_ONE_TIME_OVERLAY_CONF, CHANEEL_ONE_BOX_OVERLAY_CONF},

    {"Channel Two", CHANEEL_TWO_VIDEO_CONF, CHANEEL_TWO_OVERLAY_CONF,
        CHANEEL_TWO_TEXT_OVERLAY_CONF, CHANEEL_TWO_TIME_OVERLAY_CONF, CHANEEL_TWO_BOX_OVERLAY_CONF},

    {"Channel Three", CHANEEL_THREE_VIDEO_CONF, CHANEEL_THREE_OVERLAY_CONF,
        CHANEEL_THREE_TEXT_OVERLAY_CONF, CHANEEL_THREE_TIME_OVERLAY_CONF, CHANEEL_THREE_BOX_OVERLAY_CONF},
}

// camera map
var CaMap = CamMap_New()

// channel map
var CMap = ChMap_New()

// session map
var SMap = New()

// bitmap
var Bmap = NewBitmapSize(32)

// proxy base port
var __basePort uint64 = 8080

// server res config
var MaxSessNum int = 1
var MaxChannelNum int = 1
var MaxRtspNum int = 1
var MaxRtspSessNum int = 1
var MaxRecNum int = 1
var CameraRes int = 8294400

type resolution_info struct {
    Width  int
    Height int
}

var ResolutionOption = []resolution_info{}
var ResolutionConf = []int{}
var ResolutionList = []string{}
var EncodeOption = []int{0, 1}
var VAMResolution = resolution_info{}

// Encode list
var EncodeList = []string{"HEVC/H.265", "AVC/H.264"}

// Bitrate list
var BitRateList = []int{512000, 768000, 1000000, 1500000, 2000000, 3000000, 4000000, 6000000, 8000000, 10000000, 20000000}

// Bitrate show list
var BitRateShowList = []string{"512Kbps", "768Kbps", "1Mbps", "1.5Mbps", "2Mbps", "3Mbps", "4Mbps", "6Mbps", "8Mbps", "10Mbps", "20Mbps"}

// Fps list
var FpsList = []int{24, 30}

type HttpHeaderContent struct {
    ContentType  string
    ContentValue string
}

type DeviceConf struct {
     CamType string `json:"CamType"`
     CamResolutionRange []string `json:"CamResolutionOption"`
     VAMResolutionVal string `json:"VAMResolution"`
}


// resp http header
var HTTPHeader = HttpHeaderContent{"Content-Type", "application/json"}

// resp result
type Result struct {
    Status bool `json:"status"`
}

// resp result reason
type Result_reason struct {
    Status bool   `json:"status"`
    Reason string `json:"reason"`
}

var __resp_res = Result{false}

type RespRtspURL struct {
    Url    string `json:"url"`
    Proxy  string `json:"proxyport"`
    Status bool   `json:"status"`
}

// http client
var client = &http.Client{}

// Camera ip
var CameraIp string = "192.168.1.111"

// Enroll id
var EnrollId int = 0

// Camera ID
var CameraID int =0

// Image id
var ImageId int = 0

// Video id
var VideoId int = 0

// header
var Header string

// Onvif status
var OnvifSt bool = false

// Camera port
var CameraPort string = ":1080"

// Camera recovery
var IsRecovery bool = false

// Is wifi AP mode
var IsWifiAPMode bool = false

// audio config
type Audio struct {
    Sample_rate  int
    Num_channels int
    Bit_depth    int
    Track_codec  int
    Track_output int
    Bitrate      int
}

var AudioConf Audio
var DeviceConfig = DeviceConf{}

func Http_post(url string, reader io.Reader) *http.Response {
    // Create request
    req, err := http.NewRequest("POST", url, reader)

    req.Header.Set("Content-Type", "application/x-www-form-urlencoded; param=value")
    // send request
    resp, err := client.Do(req)

    if err != nil {
        log.Printf("Server http error:%v \n", err)
    }

    return resp
}

func Get_resp_ret(resp *http.Response) bool {
    var ret = false

    if resp == nil {
        return ret
    }

    body, err := ioutil.ReadAll(resp.Body)

    if err != nil {
        log.Printf("Server response error:%v \n", err)
    }
    log.Printf("Http response:%v \n", string(body))

    js, err := simplejson.NewJson(body)

    status, _ := js.Get("Status").String()

    if strings.Compare(status, "Success") == 0 {
        ret = true
    }
    defer resp.Body.Close()

    return ret
}

func Exist(filename string) bool {
    _, err := os.Stat(filename)
    return err == nil || os.IsExist(err)
}

func Create_video_track(chinfo Channel, trackid int, sessid int, output int) bool {
    v := url.Values{}
    v.Set("camera_id", strconv.Itoa(CameraID))
    v.Add("track_id", strconv.Itoa(trackid))
    v.Add("session_id", strconv.Itoa(sessid))
    v.Add("track_width", strconv.Itoa(ResolutionOption[chinfo.VideoConf.Resolution].Width))
    v.Add("track_height", strconv.Itoa(ResolutionOption[chinfo.VideoConf.Resolution].Height))
    v.Add("framerate", strconv.Itoa(FpsList[chinfo.VideoConf.Fps]))
    v.Add("track_codec", strconv.Itoa(EncodeOption[chinfo.VideoConf.Encode]))
    // preview
    v.Add("track_output", strconv.Itoa(output))
    v.Add("bitrate", strconv.Itoa(BitRateList[chinfo.VideoConf.Bitrate]))
    v.Add("low_power_mode", "0")

    // camera res

    log.Printf("Preview createvideotrack HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__create_video_track_URL, body)
    return Get_resp_ret(resp)
}

func Start_session(sessid int) bool {
    v := url.Values{}
    v.Add("session_id", strconv.Itoa(sessid))

    log.Printf("Preview startsession HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__start_session_URL, body)
    return Get_resp_ret(resp)
}

func Stop_session(sessid int) bool {
    v := url.Values{}
    v.Add("session_id", strconv.Itoa(sessid))
    v.Add("flush", "1")

    log.Printf("Stop session HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__stop_session_URL, body)
    return Get_resp_ret(resp)
}

func Create_session() int {
    respCs := Http_post(__create_session_URL, nil)
    log.Printf("Create session HTTP request\n")
    sid := __get_resp_sid(respCs)
    return sid
}

func __get_resp_sid(resp *http.Response) int {
    body, err := ioutil.ReadAll(resp.Body)

    if err != nil {
        log.Printf("Server response error:%v \n", err)
    }
    log.Printf("Server connect response:%v \n", string(body))

    js, err := simplejson.NewJson(body)

    sid, _ := js.Get("SessionId").Int()

    defer resp.Body.Close()

    return sid
}

func Delete_video_track(trackid int, sessid int) bool {
    v := url.Values{}
    v.Add("track_id", strconv.Itoa(trackid))
    v.Add("session_id", strconv.Itoa(sessid))
    log.Printf("Delete video track HTTP request:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__delete_video_track_URL, body)
    return Get_resp_ret(resp)
}

func Get_switch_status(r *http.Request) (bool, string) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    js, _ := simplejson.NewJson(body)

    st, _ := js.Get("switchStatus").Bool()
    conf, _ := js.Get("vamconfig").String()

    log.Printf("Switch status:%v ,conf:%v\n", st, conf)

    return st, conf
}

func Resp_ret(w http.ResponseWriter, res bool) {
    // resp json header
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)

    __resp_res.Status = res
    j, _ := json.Marshal(__resp_res)

    w.Write(j)
}

func Resp_ret_reason(w http.ResponseWriter, reason string) {
    // resp json header
    w.Header().Set(HTTPHeader.ContentType, HTTPHeader.ContentValue)

    m := &Result_reason{
        Status: false,
        Reason: reason,
    }

    j, _ := json.Marshal(m)

    w.Write(j)
}

func Get_server_info(sid int) (url string) {

    respGet := Http_post(Get_server_info_URL, nil)

    body, err := ioutil.ReadAll(respGet.Body)

    if err != nil {
        log.Printf("Server response error:%v \n", err)
    }

    if len(body) == 0 {
        log.Printf("Error resp server info is null\n")
        return
    }

    return __resp_get_server_info(body, sid)
}

func __resp_get_server_info(body []byte, sid int) (url string) {
    js, _ := simplejson.NewJson(body)

    log.Printf("Server response:%v \n", string(body))

    tracks := js.Get("Tracks")

    var rtspURL string

    for i := 0; i < 6; i++ {
        sessid, _ := tracks.GetIndex(i).Get("session_id").Int()
        if sessid == sid {
            rtspURL, _ = tracks.GetIndex(i).Get("rtsp_url").String()
        }
    }
    return rtspURL
}

func Make_resp_rtspURL(host string, rtsp_url string) string {
    u, err := url.Parse(rtsp_url)
    if err != nil {
        log.Fatal(err)
    }

    h := strings.Split(u.Host, ":")

    u.Host = strings.Replace(u.Host, h[0], host, -1)

    return u.String()
}

func Resp_url(w http.ResponseWriter, v interface{}) {
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)
    w.Write(j)
}

func Set_rtsp_proxy(state int, sess string, sessinfo Session) string {
    offset := Bmap.Find_first_zero_bit()
    log.Printf("offset is:%v\n", offset)

    Bmap.SetBit(offset, 1)

    port := strconv.FormatUint(__basePort+offset, 10)

    sessinfo.ProxyConf[state].Offset = offset
    sessinfo.ProxyConf[state].ProxyCmd = Command("rtsp_proxy", port)
    sessinfo.ProxyConf[state].Port = port

    err := sessinfo.ProxyConf[state].ProxyCmd.Start()

    if err != nil {
        log.Fatal(err)
    }

    SMap.Set(sess, sessinfo)

    return port
}

func CheckSession(r *http.Request) (sess string, ok bool) {
    var ret = false
    cookie, _ := r.Cookie("session")

    if SMap.Size() == 0 {
        log.Printf("No active session")
        return "", ret
    }
    if _, ok := SMap.Get(cookie.Value); ok {
        ret = true
    }

    return cookie.Value, ret
}

func Sync_channel_overlay_conf(chindex int, ch string) {
    chinfo, _ := CMap.ChMap_Get(ch)

    if Exist(ChConfList[chindex].Ov_Conf) {
        conf, _ := ioutil.ReadFile(ChConfList[chindex].Ov_Conf)

        jsdata, _ := simplejson.NewJson(conf)

        chinfo.OverlayType, _ = jsdata.Get("ov_type_SelectVal").Int()
    }

    if Exist(ChConfList[chindex].Ov_Time_Conf) {
        conf, _ := ioutil.ReadFile(ChConfList[chindex].Ov_Time_Conf)

        jsdata, _ := simplejson.NewJson(conf)

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

    if Exist(ChConfList[chindex].Ov_Text_Conf) {
        conf, _ := ioutil.ReadFile(ChConfList[chindex].Ov_Text_Conf)

        jsdata, _ := simplejson.NewJson(conf)

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

    if Exist(ChConfList[chindex].Ov_Box_Conf) {
        conf, _ := ioutil.ReadFile(ChConfList[chindex].Ov_Box_Conf)

        jsdata, _ := simplejson.NewJson(conf)

        chinfo.OverlayConf[2].Ov_type, _ = jsdata.Get("ov_type_SelectVal").Int()
        chinfo.OverlayConf[2].Ov_box_name, _ = jsdata.Get("ov_box_name").String()
        chinfo.OverlayConf[2].Ov_start_x, _ = jsdata.Get("ov_start_x").Int()
        chinfo.OverlayConf[2].Ov_start_y, _ = jsdata.Get("ov_start_y").Int()
        chinfo.OverlayConf[2].Ov_width, _ = jsdata.Get("ov_width").Int()
        chinfo.OverlayConf[2].Ov_height, _ = jsdata.Get("ov_height").Int()
        chinfo.OverlayConf[2].Ov_color, _ = jsdata.Get("ov_color").String()
    }

    // set Cmap
    CMap.ChMap_Set(ch, chinfo)
}

func Close_rtsp_proxy(state int, sess string) {
    // Close proxy
    sessinfo, _ := SMap.Get(sess)

    if sessinfo.ProxyConf[state].ProxyCmd != nil {
        sessinfo.ProxyConf[state].ProxyCmd.Process.Kill()

        Bmap.SetBit(sessinfo.ProxyConf[state].Offset, 0)

        w_err := sessinfo.ProxyConf[state].ProxyCmd.Wait()
        log.Printf("Command finished with error: %v", w_err)

        sessinfo.ProxyConf[state].ProxyCmd = nil
        sessinfo.ProxyConf[state].Port = ""
        SMap.Set(sess, sessinfo)
    }
}

func AddResolutionConf(resVal int) {
    ResolutionConf = append(ResolutionConf, resVal)
}

func AddResolutionOption( w int, h int) {
    ResolutionOption = append(ResolutionOption, resolution_info{w,h})
}

func AddResolutionList(resVal string) {
    ResolutionList = append(ResolutionList,resVal)
}

func SetCameraID(value int) {
    CameraID = value
}

func SetVAMResolution(width int, height int) {
    VAMResolution = resolution_info{width,height}
}

func Save_file(path string, body []byte) {
    f, err := os.Create(path)

    if err != nil {
        panic("error")
    }
    defer f.Close()

    f.Write(body)
    f.Sync()
}

func Create_audio_track(trackid int, sessid int, output int) bool {
    v := url.Values{}
    v.Set("sample_rate", strconv.Itoa(AudioConf.Sample_rate))
    v.Add("track_id", strconv.Itoa(trackid))
    v.Add("session_id", strconv.Itoa(sessid))
    v.Add("num_channels", strconv.Itoa(AudioConf.Num_channels))
    v.Add("bit_depth", strconv.Itoa(AudioConf.Bit_depth))
    v.Add("track_codec", strconv.Itoa(AudioConf.Track_codec))
    v.Add("track_output", strconv.Itoa(output))
    v.Add("bitrate", strconv.Itoa(AudioConf.Bitrate))

    log.Printf("Createaudiotrack form:%v \n", v.Encode())
    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__create_audio_track_URL, body)
    return Get_resp_ret(resp)
}

func Delete_audio_track(trackid int, sessid int) bool {
    v := url.Values{}
    v.Add("track_id", strconv.Itoa(trackid))
    v.Add("session_id", strconv.Itoa(sessid))

    body := ioutil.NopCloser(strings.NewReader(v.Encode()))

    resp := Http_post(__delete_audio_track_URL, body)
    return Get_resp_ret(resp)
}

func Get_stream_mode(r *http.Request) (bool, int) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    js, _ := simplejson.NewJson(body)

    st, _ := js.Get("switchStatus").Bool()

    val := js.Get("stream")

    if val != nil {
        stream, _ := val.Int()
        return st, stream
    }

    return st, -1
}
