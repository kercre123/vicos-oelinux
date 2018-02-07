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

package main

import "system/router"
import "system/slog"
import "system/monitor"
import "system/inotify"
import "system/recovery"
import "camera/media"
import "server"
import "sync"
import "os"
import "fmt"

const (
    WATCH_PATH string = "/data/misc/qmmf"
    __ver      string = "1.0.4"
)

var servers sync.WaitGroup

func Launch(f func()) {
    servers.Add(1)
    go func() {
        defer servers.Done()
        f()
    }()
}

func main() {
    Launch(func() {
        l := len(os.Args)

        if l == 2 {
            if os.Args[l-1] == "-v" {
                fmt.Println(__ver)
                os.Exit(0)
            } else {
                fmt.Println("Unsupported")
                os.Exit(0)
            }
        }
    })

    Launch(func() {
        slog.InitServerLog()
        server.InitCamera()
    })

    Launch(func() {
        monitor.Monitor_camera_ip()
    })

    Launch(func() {
        dirs := []string{WATCH_PATH}
        inotify.InotifyLoop(dirs, media.Callback)
    })

    Launch(func() {
        router.ListenHttp()
    })

    Launch(func() {
        recovery.Monitor_qmmf_webserver()
    })

    servers.Wait()
}
