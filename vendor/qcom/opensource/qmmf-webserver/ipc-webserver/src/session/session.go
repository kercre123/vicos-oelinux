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
package session

/*
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <time.h>
#include <string.h>

char sessionid[256];

static char* generateSessionId()
{
    struct timespec tspec;
    memset(&tspec, 0, sizeof(tspec));
    // get monotonic time
    syscall(SYS_clock_gettime, CLOCK_MONOTONIC_RAW, &tspec);
    int sec = tspec.tv_sec;
    int msec = tspec.tv_nsec;
    sprintf(sessionid, "%x%x", sec,msec);
    return sessionid;
}
*/
import "C"
import "github.com/bitly/go-simplejson"
import "net/http"
import "io/ioutil"
import "strings"
import "log"
import . "system/smap"
import . "common"
import . "camera/preview"
import . "camera/recording"
import . "camera/overlay"
import . "camera/vam"
import "server"

// const define
const (
    USERNAME string = "admin"
    USERPWD  string = "admin"
)

func __resp_cookie(sessionid string, w http.ResponseWriter) {
    // set cookie
    cookie := http.Cookie{
        Name:  "session",
        Value: sessionid,
    }

    http.SetCookie(w, &cookie)
}

func __authenticate_login(body []byte) bool {
    var ret = false

    js, _ := simplejson.NewJson(body)

    // get username and userpwd
    username, _ := js.Get("username").String()
    userpwd, _ := js.Get("userpwd").String()
    if strings.Compare(username, USERNAME) == 0 &&
        strings.Compare(userpwd, USERPWD) == 0 {
        ret = true
    }
    return ret
}

func __save_session_map(sid string) {
    // put sessmap
    SMap.Set(sid, Session{sid, 0, false, false, false, false, [5]RtspProxyConf{{nil, 0, "", false},
        {nil, 0, "", false}, {nil, 0, "", false}, {nil, 0, "", false},{nil, 0, "", false}}})
}

func LoginHandler(w http.ResponseWriter, r *http.Request) {
    log.Printf("Login session num:%v \n", SMap.Size())
    log.Printf("Login maxsession :%v \n", MaxSessNum)

    if SMap.Size() >= MaxSessNum {
        Resp_ret(w, false)
        return
    }

    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    if len(body) == 0 {
        log.Printf("Error username passwd is null\n")
        Resp_ret(w, false)
        return
    }

    if !__authenticate_login(body) {
        Resp_ret(w, false)
        return
    }

    if SMap.Size() < 1 {
        server.InitConnection()
    }

    // generate session
    sessionid := C.GoString(C.generateSessionId())
    log.Printf("Login sessionid:%v \n", sessionid)

    // save session
    __save_session_map(sessionid)
    // resp cookie
    __resp_cookie(sessionid, w)
    // resp ok
    Resp_ret(w, true)
}

func LogoutHandler(w http.ResponseWriter, r *http.Request) {
    // get session
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
        server.DeInitConnection()
    }

    log.Printf("Logout smap size:%v \n", SMap.Size())

    Resp_ret(w, true)
}
