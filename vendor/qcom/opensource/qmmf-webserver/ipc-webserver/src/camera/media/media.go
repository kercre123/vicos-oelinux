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

package media

import (
    . "common"
    "encoding/json"
    "github.com/bitly/go-simplejson"
    "io/ioutil"
    "log"
    "math"
    "net/http"
    "net/url"
    "os"
    "path"
    "path/filepath"
    "strconv"
    "strings"
    "sync"
    "sort"
    "regexp"
)

type Files_conf []map[string]string

// video list
var video_list = make(Files_conf, 0)

// images list
var images_list = make(Files_conf, 0)

// sync mutex
var mutex = &sync.Mutex{}

// video page num
var VPNum int = 10

// image page num
var IPNum int = 10

// sort File list
var videoFiles []string
var imageFiles []string

type RespMedia struct {
    Files        []map[string]string `json:"files"`
    TotalPageNum int                 `json:"totalPageNum"`
    Status       bool                `json:"status"`
}

type Files []string

func (a Files) Len() int      { return  len(a) }
func (a Files) Swap(i, j int) { a[i], a[j] = a[j], a[i] }

type ByMtime struct{ Files }

func (a ByMtime) Less(i, j int) bool {
    iFi, _ := os.Stat(a.Files[i])
    jFi, _ := os.Stat(a.Files[j])
    return iFi.ModTime().Before(jFi.ModTime())
}

func Callback(files, dirs, events []string) {
    nr := len(files)

    for i := 0; i < nr; i++ {
        __deal_file_event(files[i], dirs[i], events[i])
    }
}

func __deal_file_event(file, dir, event string) {
    if event == "CREATE" {
        if path.Ext(file) == ".mp4" {
            __insert_video_list(file)
        }

        if path.Ext(file) == ".jpeg" ||
            path.Ext(file) == ".jpg" {
            __insert_images_list(file)
        }
        return
    }

    if event == "DELETE" {
        if path.Ext(file) == ".mp4" {
            __delete_video_list(file)
        }

        if path.Ext(file) == ".jpeg" ||
            path.Ext(file) == ".jpg" {
            __delete_images_list(file)
        }
        return
    }

}

func Scan_media_file() {
    // clear list
    video_list = video_list[0:0]
    images_list = images_list[0:0]
    videoFiles = videoFiles[0:0]
    imageFiles = videoFiles[0:0]

    // scan media file
    filepath.Walk(MEDIA_DIR,
        func(file string, f os.FileInfo, err error) error {

            if f == nil {
                return err
            }

            if f.IsDir() {
                return nil
            }

            // video
            if path.Ext(file) == ".mp4" {
                videoFiles = append(videoFiles, file)
                return nil
            }

            // image
            if path.Ext(file) == ".jpeg" ||
                path.Ext(file) == ".jpg" {

                matched, _ := regexp.MatchString("dist", file)

                if (!matched) {
                    imageFiles = append(imageFiles, file)
                }
                return nil
            }
            return nil
        })
        // sort by time
        sort.Sort(ByMtime{videoFiles})
        sort.Sort(ByMtime{imageFiles})

        // insert list
        for _, value := range videoFiles {
            filename := strings.Replace(value, MEDIA_DIR, "", -1)
            __insert_video_list(filename)
        }

        for _, value := range imageFiles {
            filename := strings.Replace(value, MEDIA_DIR, "", -1)
            __insert_images_list(filename)
        }

}

func __insert_video_list(filename string) {
    log.Printf("insert videolist file = %v\n", filename)
    mutex.Lock()

    file := make(map[string]string)

    var url string = Header + CameraIp + CameraPort + "/media/" + filename
    file["url"] = url
    file["filename"] = filename
    video_list = append(video_list, file)

    mutex.Unlock()
}

func __insert_images_list(filename string) {
    log.Printf("insert imageslist file = %v\n", filename)
    mutex.Lock()

    file := make(map[string]string)

    var url string = Header + CameraIp + CameraPort + "/media/" + filename
    file["url"] = url
    file["filename"] = filename
    images_list = append(images_list, file)

    mutex.Unlock()
}

func __delete_video_list(filename string) {
    log.Printf("delete videolist file = %v\n", filename)

    for index, value := range video_list {
        if value["filename"] == filename {
            __delete_video_value(index)
            break
        }
    }

}

func __delete_video_value(i int) {
    log.Printf("delete index:%v \n", i)

    video_list = append(video_list[:i], video_list[i+1:]...)

    log.Printf("Length video list:%v \n", len(video_list))
}

func __delete_images_list(filename string) {
    log.Printf("delete imageslist file = %v\n", filename)

    for index, value := range images_list {
        if value["filename"] == filename {
            __delete_image_value(index)
            break
        }
    }
}

func __delete_image_value(i int) {
    log.Printf("delete index:%v \n", i)

    images_list = append(images_list[:i], images_list[i+1:]...)

    log.Printf("Length video list:%v \n", len(images_list))
}

func __resp_media(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    log.Printf(string(j))

    w.Write(j)
}

func __replay_get(w http.ResponseWriter, r *http.Request) {

    k, _ := url.ParseQuery(r.URL.RawQuery)

    v := k["pageNumber"][0]

    pageNum, _ := strconv.Atoi(v)

    size := len(video_list)

    var totalPageNum float64 = float64(size) / float64(10)

    log.Printf("pageNum:%d, totalPageNum: %f \n", pageNum, totalPageNum)

    var resplist = make(Files_conf, 0)

    ret := size - (pageNum * VPNum)

    if ret < 0 {
        resplist = video_list[(pageNum*VPNum - VPNum):size]
    } else {
        resplist = video_list[(pageNum*VPNum - VPNum):(pageNum * VPNum)]
    }

    m := &RespMedia{
        Files:        resplist,
        TotalPageNum: int(math.Ceil(totalPageNum)),
        Status:       true,
    }
    __resp_media(w, m)
}

func __save_conf(path string, conf []byte) {
    err := ioutil.WriteFile(path, conf, 0666)
    if err != nil {
        panic("error")
    }
}

func __replay_post(w http.ResponseWriter, r *http.Request) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    __save_conf(VIDEO_CONF, body)

    Resp_ret(w, true)
}

func ReplayHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "GET" {
        __replay_get(w, r)
        return
    }

    if r.Method == "POST" {
        __replay_post(w, r)
    }
}

func __images_get(w http.ResponseWriter, r *http.Request) {
    k, _ := url.ParseQuery(r.URL.RawQuery)

    v := k["pageNumber"][0]

    pageNum, _ := strconv.Atoi(v)

    size := len(images_list)

    var totalPageNum float64 = float64(size) / float64(10)

    log.Printf("pageNum:%d, totalPageNum: %f \n", pageNum, totalPageNum)

    var resplist = make(Files_conf, 0)

    if size-(pageNum*VPNum) < 0 {
        resplist = images_list[(pageNum*IPNum - IPNum):size]
    } else {
        resplist = images_list[(pageNum*IPNum - IPNum):(pageNum * IPNum)]
    }

    m := &RespMedia{
        Files:        resplist,
        TotalPageNum: int(math.Ceil(totalPageNum)),
        Status:       true,
    }
    __resp_media(w, m)
}

func __images_post(w http.ResponseWriter, r *http.Request) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    __save_conf(IMAGE_CONF, body)

    Resp_ret(w, true)
}

func ImagesHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "GET" {
        __images_get(w, r)
        return
    }

    if r.Method == "POST" {
        __images_post(w, r)
    }
}

func __delete_post(w http.ResponseWriter, r *http.Request) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    log.Printf("body :%v\n", string(body))
    js, _ := simplejson.NewJson(body)

    files, _ := js.Get("files").Array()

    var str string = Header + CameraIp + CameraPort + "/media/"

    for _, file := range files {
        newdi, _ := file.(map[string]interface{})
        url := newdi["url"]
        filename := strings.Replace(url.(string), str, "", -1)
        path := MEDIA_DIR + filename
        os.Remove(path)
    }

    // default true
    Resp_ret(w, true)
}

func VideoDeleteHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "POST" {
        __delete_post(w, r)
    }
}

func ImageDeleteHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "POST" {
        __delete_post(w, r)
    }
}
