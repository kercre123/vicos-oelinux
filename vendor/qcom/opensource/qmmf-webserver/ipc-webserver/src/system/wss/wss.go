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

package wss

import (
    . "camera/overlay"
    . "camera/preview"
    . "camera/recording"
    . "camera/vam"
    . "common"
    . "server"
    "github.com/bitly/go-simplejson"
    "github.com/gorilla/websocket"
    "log"
    "net/http"
    "strings"
    "sync"
    "time"
)

type wss_entry struct {
    lwss      *websocket.Conn
    sess      string
    keepalive *time.Timer
    countdown *time.Timer
}

var upgrader = websocket.Upgrader{} // use default options

var wss_list = make([]*wss_entry, 0)

var mutex = &sync.Mutex{}

func __insert_wss_list(lwss *websocket.Conn, sess string) {
    mutex.Lock()
    var entry = new(wss_entry)

    entry.lwss = lwss
    entry.sess = sess
    entry.keepalive = time.NewTimer(time.Second * 35)

    wss_list = append(wss_list, entry)

    go keeplive_timer(entry)

    mutex.Unlock()
}

func __delete_wss_list(index int) {
    mutex.Lock()
    wss_list = append(wss_list[:index], wss_list[index+1:]...)

    log.Printf("Length wss list:%v \n", len(wss_list))
    mutex.Unlock()
}

func wss_assoc(lwss *websocket.Conn, message []byte) {
    if len(message) == 0 {
        log.Printf("Error wss message is null\n")
        return
    }

    js, _ := simplejson.NewJson(message)

    cookie, _ := js.Get("cookie").String()

    s := strings.Split(cookie, "=")

    if len(s) < 2 {
        log.Printf("cookie format error :%v \n", len(s))
        return
    }

    if s[1] == "" {
        return
    }

    ret, entry := wss_sess_lookup(s[1])

    if ret == -1 {
        __insert_wss_list(lwss, s[1])
        log.Printf("Length wss list:%v \n", len(wss_list))
    } else {
        if entry.lwss == lwss {
            entry.keepalive.Reset(time.Second * 35)
        } else {
            // reset timer
            if entry.countdown != nil {
                entry.countdown.Reset(0 * time.Second)
                entry.countdown.Stop()
            }

            if entry.keepalive != nil {
                entry.keepalive.Reset(0 * time.Second)
                entry.keepalive.Stop()
            }

            // delete entry
            __delete_wss_list(ret)
            // insert list
            __insert_wss_list(lwss, s[1])
            log.Printf("Length wss list:%v \n", len(wss_list))
        }
    }
}

func keeplive_timer(entry *wss_entry) {
    select {
    case <-entry.keepalive.C:
        log.Printf("keepalive timeout:%v \n", entry.sess)
        __deal_timer(entry)
    }
}

func __deal_timer(entry *wss_entry) {
    wss_cleanup(entry.lwss)
}

func countdown_timer(entry *wss_entry) {
    select {
    case <-entry.countdown.C:
        log.Printf("countdown timeout:%v \n", entry.sess)
        __deal_timer(entry)
    }
}

func wss_sess_lookup(sess string) (int, *wss_entry) {
    for index, value := range wss_list {
        if value != nil {
            if value.sess == sess {
                return index, value
            }
        }
    }
    return -1, nil
}

func wss_lwss_lookup(lwss *websocket.Conn) (int, *wss_entry) {
    for index, value := range wss_list {
        if value != nil {
            if value.lwss == lwss {
                return index, value
            }
        }
    }
    return -1, nil
}

func __timeout_wss_cleanup(lwss *websocket.Conn) {
    _, entry := wss_lwss_lookup(lwss)

    if entry == nil {
        return
    }

    entry.countdown = time.NewTimer(time.Millisecond * 1500)

    go countdown_timer(entry)
}

func wss_cleanup(lwss *websocket.Conn) {
    ret, entry := wss_lwss_lookup(lwss)

    if ret == -1 {
        return
    }

    __wss_cleanup(ret, entry.sess)
    // reset timer
    entry.keepalive.Reset(0 * time.Second)
    entry.keepalive.Stop()
}

func __wss_cleanup(index int, sess string) {

    __delete_wss_list(index)

    if sess != "" {
        __session_close_notify(sess)
    }
}

func __session_close_notify(sess string) {
    log.Printf("Session close:%v \n", sess)

    sessinfo, ok := SMap.Get(sess)

    if !ok {
        return
    }

    // get session channel
    ch := sessinfo.Channel
    // get channel info
    chinfo, _ := CMap.ChMap_Get(ChList[ch])

    if sessinfo.OvStatus {
        // Overlay Off
        Switch_overlay_off(chinfo, ch, sess)
    }

    chinfo, _ = CMap.ChMap_Get(ChList[ch])

    for index := FR; index < MAX; index++ {
        if sessinfo.ProxyConf[index].Status {
            // remove vam config
            chinfo, ch = Vam_tmp_one_streaming(sessinfo)

            Vam_remove_config(chinfo, ch, index)

            chinfo, ch = Vam_tmp_one_streaming(sessinfo)

            // vam off
            Switch_vam_off(chinfo, sess, ch)
        }
    }

    // ch revert
    ch = sessinfo.Channel
    chinfo, _ = CMap.ChMap_Get(ChList[ch])

    if sessinfo.PreStatus {
        // Preview off
        Switch_preview_off(chinfo, sess, ch)
    }

    chinfo, _ = CMap.ChMap_Get(ChList[ch])

    if sessinfo.RecStatus {
        // Rec off
        Switch_recording_off(chinfo, sess, ch)
    }

    // delete session
    SMap.Delete(sess)

    if SMap.Size() < 1 {
        DeInitConnection()
    }

    log.Printf("Close smap size:%v \n", SMap.Size())
}

func Sess_logout(sess string) {
    for _, value := range wss_list {
        if value != nil {
            if value.sess == sess {
                value.sess = ""
            }
        }
    }
    return
}

func Async(w http.ResponseWriter, r *http.Request) {
    c, err := upgrader.Upgrade(w, r, nil)

    if err != nil {
        log.Print("upgrade:", err)
        return
    }
    defer c.Close()

    for {
        mt, message, err := c.ReadMessage()
        if err != nil {
            log.Printf("error: %v, user-agent: %v", err, r.Header.Get("User-Agent"))
            // wss close
            __timeout_wss_cleanup(c)
            return
        }

        if err != nil {
            log.Println("read:", err)
            break
        }
        // assoc
        wss_assoc(c, message)

        err = c.WriteMessage(mt, message)
        if err != nil {
            log.Println("write:", err)
            break
        }
    }
}
