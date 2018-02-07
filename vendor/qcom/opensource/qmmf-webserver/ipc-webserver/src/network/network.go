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

package network

import (
    "bytes"
    . "common"
    "encoding/json"
    "github.com/bitly/go-simplejson"
    "io/ioutil"
    "log"
    "net/http"
    "os"
    . "os/exec"
    "strconv"
    "strings"
)

// network mode list
var NetWorkModeList = []string{"DHCP", "Static"}

// network mode list
var WiFiModeList = []string{"AP"}

type Net_conf struct {
    Mode int
    Ip   string
    Mask string
    RangeStart string
    RangeEnd   string
}

type AP_conf struct {
    Mode     int
    Ssid     string
    Password string
}

var NetConf Net_conf
var ApConf AP_conf

const (
    __CLEAN_IP_CMD string = "ifconfig br0 0.0.0.0"
)

type RespDHCP struct {
    ModeSelectVal int      `json:"modeSelectVal"`
    Mode          []string `json:"mode"`
    Status        bool     `json:"status"`
}

type RespStatic struct {
    ModeSelectVal int      `json:"modeSelectVal"`
    Mode          []string `json:"mode"`
    Ip            string   `json:"ip"`
    Mask          string   `json:"mask"`
    RangeStart    string   `json:"range_start"`
    RangeEnd      string   `json:"range_end"`
    Status        bool     `json:"status"`
}

type RespAP struct {
    ModeSelectVal int      `json:"modeSelectVal"`
    Mode          []string `json:"mode"`
    Ssid          string   `json:"ssid"`
    Password      string   `json:"password"`
    Status        bool     `json:"status"`
}

var __dhcpCmd *Cmd = nil
var __hostapdCmd *Cmd = nil

func __exec_dhcp_cmd() error {

    if Exist("/data/run/dhcpcd-br0.pid") {
        __exec_rm_cmd("rm -rf /data/run/dhcpcd-br0.pid");
    }

    __dhcpCmd = Command("dhcpcd", "br0", "-b", "-t", "0")

    var out bytes.Buffer
    var stderr bytes.Buffer

    __dhcpCmd.Stdout = &out
    __dhcpCmd.Stderr = &stderr

    s_err := __dhcpCmd.Start()

    if s_err != nil {
        log.Printf("dhcpcd start error: %v", stderr.String())
    }

    // wait
    w_err := __dhcpCmd.Wait()

    log.Printf("Command finished with error: %v:%v", w_err, stderr.String())

    return s_err
}

func __exec_cleanip_cmd() error {
    exec_cmd := Command("/bin/sh", "-c", __CLEAN_IP_CMD)

    err := exec_cmd.Run()

    if err != nil {
        log.Printf("Error exec cleanip cmd: %v", err)
    }

    return err
}

func __exec_static_cmd(cmd string) error {
    log.Printf("exec static cmd: %v", cmd)

    exec_cmd := Command("/bin/sh", "-c", cmd)

    err := exec_cmd.Run()

    if err != nil {
        log.Printf("Error exec ipconf cmd: %v", err)
    }

    return err
}

func __exec_kill_dnsmasq() error {
    pid := __exec_get_pid_cmd("cat /data/br0_dnsmasq.pid")

    pro := __exec_kill_cmd(pid)

    err := pro.Kill()
    log.Printf("Command kill process err: %v", err)

    err = pro.Release()
    log.Printf("Command Release process err: %v", err)

    if err != nil {
        log.Printf("Error exec cleanip cmd: %v", err)
    }

    return err
}

func Deal_dhcp_cmd() bool {
    if __exec_cleanip_cmd() != nil {
        return false
    }

    if __exec_kill_dnsmasq() != nil {
        return false
    }

    if __exec_dhcp_cmd() != nil {
        return false
    }

    return true
}

func __get_mode(body []byte) int {
    if len(body) == 0 {
        log.Printf("Error mode is null\n")
        return -1
    }

    js, _ := simplejson.NewJson(body)

    mode, _ := js.Get("modeSelectVal").Int()

    return mode
}

func __deal_dhcp_mode(w http.ResponseWriter) bool {
    return Deal_dhcp_cmd()
}

func __get_rtsp_ip_mask(body []byte) (staic_ip string, static_mask string) {
    if len(body) == 0 {
        log.Printf("Error rtsp ip mask null\n")
        return
    }

    js, _ := simplejson.NewJson(body)

    ip, _ := js.Get("ip").String()
    mask, _ := js.Get("mask").String()

    return ip, mask
}

func __get_dhcp_range(body []byte) (range_start string, range_end string) {
    if len(body) == 0 {
        log.Printf("Error dhcp range null\n")
        return
    }

    js, _ := simplejson.NewJson(body)

    start, _ := js.Get("range_start").String()
    end, _ := js.Get("range_end").String()

    return start, end
}

func Deal_static_cmd(body []byte) bool {
    if NetConf.Mode == 1 {
        if __exec_kill_dnsmasq() != nil {
            return false
        }
    }

    ip, mask := __get_rtsp_ip_mask(body)

    var __ip_conf_cmd string = "ifconfig br0" + " " + ip + " " + "netmask" + " " + mask

    if __exec_static_cmd(__ip_conf_cmd) != nil {
        return false
    }

    start, end := __get_dhcp_range(body)

    var dnsmasq_cmd string = "dnsmasq --conf-file=/data/dnsmasq.conf --dhcp-leasefile=/data/dnsmasq.leases --addn-hosts=/data/hosts --pid-file=/data/br0_dnsmasq.pid -i br0 -I lo -z --dhcp-range=br0,"+start+","+end+",255.255.255.0,43200 --dhcp-hostsfile=/data/dhcp_hosts"

    if __exec_static_cmd(dnsmasq_cmd) != nil {
        return false
    }

    // save conf
    NetConf.Ip = ip
    NetConf.Mask = mask
    NetConf.RangeStart = start
    NetConf.RangeEnd = end

    return true
}

func __deal_static_mode(w http.ResponseWriter, body []byte) bool {
    if __dhcpCmd != nil {
        log.Printf("Command kill process id: %v", __dhcpCmd.Process.Pid)
        __dhcpCmd.Process.Kill()
    }

    if NetConf.Mode == 0 {
        pid := __exec_get_pid_cmd("cat /var/run/dhcpcd-br0.pid")

        pro := __exec_kill_cmd(pid)

        err := pro.Kill()
        log.Printf("Command kill process err: %v", err)

        err = pro.Release()
        log.Printf("Command Release process err: %v", err)
    }

    return Deal_static_cmd(body)
}

func __save_conf(path string, conf []byte) {
    f, err := os.Create(path)

    if err != nil {
        panic("error")
    }
    defer f.Close()

    f.Write(conf)
    f.Sync()
}

func __network_post(w http.ResponseWriter, r *http.Request) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    mode := __get_mode(body)

    if mode == -1 {
        log.Printf("Error network mode error\n")
        Resp_ret(w, false)
        return
    }

    log.Printf("NetWork mode: %v", mode)

    if mode == 0 {
        if !__deal_dhcp_mode(w) {
            Resp_ret(w, false)
            return
        }
    }

    if mode == 1 {
        if !__deal_static_mode(w, body) {
            Resp_ret(w, false)
            return
        }
    }

    NetConf.Mode = mode
    // save conf
    __save_conf(NET_CONF, body)

    // flush smap
    size := SMap.Flush()
    log.Printf("Smap remove size: %v", size)

    Resp_ret(w, true)
}

func __resp_network(w http.ResponseWriter, v interface{}) {
    // set header json
    w.Header().Set("Content-Type", "application/json")
    j, _ := json.Marshal(v)

    w.Write(j)
}

func __network_get(w http.ResponseWriter) {
    if NetConf.Mode == 0 {
        // resp channel json
        m := &RespDHCP{
            ModeSelectVal: NetConf.Mode,
            Mode:          NetWorkModeList,
            Status:        true,
        }
        __resp_network(w, m)
        return
    }

    if NetConf.Mode == 1 {
        // resp channel json
        m := &RespStatic{
            ModeSelectVal: NetConf.Mode,
            Mode:          NetWorkModeList,
            Ip:            NetConf.Ip,
            Mask:          NetConf.Mask,
            RangeStart:    NetConf.RangeStart,
            RangeEnd:      NetConf.RangeEnd,
            Status:        true,
        }
        __resp_network(w, m)
        return
    }
}

func NetWorkHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "GET" {
        __network_get(w)
        return
    }

    if r.Method == "POST" {
        log.Printf("SMap size: %v", SMap.Size())
        if SMap.Size() > 1 {
            Resp_ret(w, false)
            return
        }
        __network_post(w, r)
    }
}

func __exec_wifi_change_cmd(cmd string) error {
    log.Printf("exec wifi change cmd: %v", cmd)

    exec_cmd := Command("/bin/sh", "-c", cmd)

    err := exec_cmd.Run()

    if err != nil {
        log.Printf("Error exec ipconf cmd: %v", err)
    }

    return err
}

func Exec_wlan0_cmd(cmd string) error {
    exec_cmd := Command("ifconfig", "wlan0", cmd)

    err := exec_cmd.Run()

    if err != nil {
        log.Printf("Error exec wlan0 cmd: %v", err)
    }

    return err
}

func __exec_hostapd_cmd() {
    log.Printf("exec hostapd cmd\n")

    __hostapdCmd = Command("hostapd", "-B", "-P", "/var/run/hostapd.wlan0.pid", "/data/misc/wifi/hostapd.conf")

    stdout, err := __hostapdCmd.StdoutPipe()

    if err != nil {
        log.Fatal(err)
    }
    defer stdout.Close()

    s_err := __hostapdCmd.Start()

    if err != nil {
        log.Printf("Error exec hostapd cmd: %v", s_err)
    }

    opBytes, err := ioutil.ReadAll(stdout)
    if err != nil {
        log.Fatal(err)
    }
    log.Printf(string(opBytes))

    __hostapdCmd.Wait()
}

func Exec_hostapd_cmd() {
    __exec_hostapd_cmd()
}

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

func __exec_kill_cmd(pid string) *os.Process {
    p, _ := strconv.Atoi(pid)
    pro, _ := os.FindProcess(p)
    return pro
}

func __exec_rm_cmd(cmd string) error {
    exec_cmd := Command("/bin/sh", "-c", cmd)

    err := exec_cmd.Run()

    if err != nil {
        log.Printf("Error exec cleanip cmd: %v", err)
    }

    return err
}

func __deal_wifi_ap_cmd(w http.ResponseWriter, body []byte) bool {
    Exec_wlan0_cmd("down")

    pid := __exec_get_pid_cmd("cat /var/run/hostapd.wlan0.pid")

    pro := __exec_kill_cmd(pid)

    err := pro.Kill()
    log.Printf("Command kill process err: %v", err)

    err = pro.Release()
    log.Printf("Command Release process err: %v", err)

    __exec_rm_cmd("rm -rf /data/misc/wifi/hostapd/")

    ssid, passwd := __get_wifi_ssid_passwd(body)

    if ssid == "" || passwd == "" {
        log.Printf("Error ssid or passwd is nulla\n")
        return false
    }

    var __change_ssid_cmd string = "sed -i '/^ssid=/s/=.*$/=" + ssid + "/' /data/misc/wifi/hostapd.conf"
    var __change_passwd_cmd string = "sed -i '/^wpa_passphrase=/s/=.*$/=" + passwd + "/' /data/misc/wifi/hostapd.conf"

    if __exec_wifi_change_cmd(__change_ssid_cmd) != nil {
        return false
    }

    if __exec_wifi_change_cmd(__change_passwd_cmd) != nil {
        return false
    }

    Exec_hostapd_cmd()

    Exec_wlan0_cmd("up")

    // save conf
    ApConf.Ssid = ssid
    ApConf.Password = passwd

    return true
}

func __get_wifi_ssid_passwd(body []byte) (string, string) {
    if len(body) == 0 {
        log.Printf("Error rtsp ssid passwd null\n")
        return "", ""
    }

    js, _ := simplejson.NewJson(body)

    ssid, _ := js.Get("ssid").String()
    passwd, _ := js.Get("password").String()

    return ssid, passwd
}

func __deal_wifi_ap_mode(w http.ResponseWriter, body []byte) {
    if !__deal_wifi_ap_cmd(w, body) {
        Resp_ret(w, false)
        return
    }
}

func __wifi_post(w http.ResponseWriter, r *http.Request) {
    body, err := ioutil.ReadAll(r.Body)

    if err != nil {
        panic("error")
    }

    mode := __get_mode(body)

    // save conf
    __save_conf(WIFI_CONF, body)

    log.Printf("Wifi mode: %v", mode)

    // flush smap
    size := SMap.Flush()
    log.Printf("Smap remove size: %v", size)

    // AP
    if mode == 0 {
        __deal_wifi_ap_mode(w, body)
        ApConf.Mode = 0
        Resp_ret(w, true)
        return
    }
}

func __wifi_get(w http.ResponseWriter) {
    if ApConf.Mode == 0 {
        // resp channel json
        m := &RespAP{
            ModeSelectVal: ApConf.Mode,
            Mode:          WiFiModeList,
            Ssid:          ApConf.Ssid,
            Password:      ApConf.Password,
            Status:        true,
        }
        __resp_network(w, m)
        return
    }
}

func WiFiHandler(w http.ResponseWriter, r *http.Request) {
    _, ok := CheckSession(r)

    if !ok {
        Resp_ret(w, false)
        return
    }

    if r.Method == "GET" {
        __wifi_get(w)
        return
    }

    if r.Method == "POST" {
        if SMap.Size() > 1 {
            Resp_ret(w, false)
            return
        }
        __wifi_post(w, r)
    }
}
