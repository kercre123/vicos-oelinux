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

package recovery

import (
    "log"
    "strings"
    . "os/exec"
    "bytes"
    "time"
    . "common"
    "os"
)


func __exec_get_pid_cmd(getcmd string) string {
    cmd := Command("/bin/sh", "-c", getcmd)

    var out bytes.Buffer
    cmd.Stdout = &out

    err := cmd.Run()

    if err != nil {
        log.Printf("Error exec getpid cmd: %v", err)
    }

    return strings.Replace(out.String(), "\n", "", -1)
}

func Monitor_qmmf_webserver() {
    pid := __exec_get_pid_cmd("ps -ef | grep qmmf-webserver | grep -v grep | awk '{print $1}'")

    log.Printf("qmmf-webserver pid:%v\n", pid)

    for {
        // sleep 1s
        time.Sleep(1000 * time.Millisecond)

        newpid := __exec_get_pid_cmd("ps -ef | grep qmmf-webserver | grep -v grep | awk '{print $1}'")

        if pid != newpid {
            log.Printf("qmmf-webserver newpid:%v\n", newpid)

            Save_file(TMP_CAMERA_IP_FILE, []byte(CameraIp))

            cmd := Command("/usr/bin/ipc-webserver")
            s_err := cmd.Start()

            if s_err != nil {
                log.Printf("s_err:%v\n", s_err)
            }

            os.Exit(0)
            break
        }
    }
}
