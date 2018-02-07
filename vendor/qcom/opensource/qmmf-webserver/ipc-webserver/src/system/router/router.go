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

package router

import (
    "camera/channel"
    "camera/media"
    "camera/overlay"
    "camera/params"
    "camera/photo"
    "camera/preview"
    "camera/recording"
    "camera/vam"
    "camera/video"
    "camera/onvif"
    . "common"
    "crypto/tls"
    "crypto/x509"
    "github.com/gorilla/mux"
    "io/ioutil"
    "log"
    "net/http"
    "network"
    "session"
    "strings"
    "sync"
    "system/wss"
    "net/url"
)

// resp http header
var RespHeader = HttpHeaderContent{"Content-Type", "application/octet-stream"}

// sync mutex
var Smutex = &sync.Mutex{}

type fn func(http.ResponseWriter, *http.Request)

//static file
const (
    // dir
    __static_dir = Webserver_dir + "/dist/"

    // https cer
    __ca_crt  = Webserver_dir + "/ca.crt"
    __ipc_crt = Webserver_dir + "/ipc.crt"
    __ipc_key = Webserver_dir + "/ipc.key"
)

var __static_fileServer http.Handler
var __media_fileServer http.Handler

var route = map[string]fn{
    "/async":         wss.Async,
}

var sroute = map[string]fn{
    // sync
    "/login":           session.LoginHandler,
    "/logout":          session.LogoutHandler,
    "/channel":         channel.ChannelHandler,
    "/video":           video.VideoHandler,
    "/params":          params.ParamsHandler,
    "/network":         network.NetWorkHandler,
    "/wifi":            network.WiFiHandler,
    "/replay":          media.ReplayHandler,
    "/images":          media.ImagesHandler,
    "/videodelete":     media.VideoDeleteHandler,
    "/imagedelete":     media.ImageDeleteHandler,
    "/overlayconfig":   overlay.OverlayConfigHandler,
    "/onvif":           onvif.OnvifHandler,
    "/captureimage":    photo.PhotoHandler,
    "/preview":         preview.PreviewHandler,
    "/recording":       recording.RecordingHandler,
    "/vam":             vam.VamHandler,
    "/vamconfig":       vam.VamConfigHandler,
    "/vamremoveconfig": vam.VamRemoveConfigHandler,
    "/vamenroll":       vam.VamEnrollHandler,
    "/overlay":         overlay.OverlayHandler,
}

func loadCerts() (*x509.CertPool, tls.Certificate) {
    caFile := __ca_crt
    certFile := __ipc_crt
    keyFile := __ipc_key

    cert, err := tls.LoadX509KeyPair(certFile, keyFile)
    if err != nil {
        panic(err)
    }

    pem, err := ioutil.ReadFile(caFile)
    if err != nil {
        panic(err)
    }

    ca := x509.NewCertPool()
    if !ca.AppendCertsFromPEM(pem) {
        panic("Cannot load ca")
    }

    return ca, cert
}

func makeTLSConfig(ca *x509.CertPool, cert tls.Certificate) *tls.Config {
    config := &tls.Config{
        RootCAs:    ca,
        MinVersion: tls.VersionTLS12,
    }

    config.Certificates = make([]tls.Certificate, 1)
    config.Certificates[0] = cert

    return config
}

func __dispatcher(w http.ResponseWriter, r *http.Request) {
    if h, ok := sroute[r.URL.Path]; ok {
        Smutex.Lock()
        log.Printf("Router __dispatcher url:%v \n", r.URL.Path)
        h(w, r)
        Smutex.Unlock()
    } else {
        dispatcher(w, r)
    }
}

func dispatcher(w http.ResponseWriter, r *http.Request) {
    log.Printf("Router dispatcher url:%v \n", r.URL.Path)

    if h, ok := route[r.URL.Path]; ok {
        h(w, r)
    } else if strings.HasPrefix(r.URL.Path, "/media") {
        // for web download
        if r.URL.RawQuery != "" {
            k, _ := url.ParseQuery(r.URL.RawQuery)

            v := k["action"][0]

            log.Printf("media query:%v \n", v)
            filename := strings.Replace(r.URL.Path, "/media/", "", -1)
            log.Printf("media filename:%v \n", filename)
            if v == "download" {
                w.Header().Set("Content-Disposition", "attachment; filename="+filename)
                w.Header().Set(RespHeader.ContentType, RespHeader.ContentValue)
            }
        }

        // remove "/media"
        r.URL.Path = strings.Replace(r.URL.Path, "/media", "", -1)

        // media file server
        __media_fileServer.ServeHTTP(w, r)
    } else {
        // static file server
        __static_fileServer.ServeHTTP(w, r)
    }
}

func ListenHttps() {
    Header = "https://"

    ca, cert := loadCerts()
    config := makeTLSConfig(ca, cert)

    listener, err := tls.Listen("tcp4", ":https", config)
    if err != nil {
        panic(err)
    }

    // static file server
    __static_fileServer = http.FileServer(http.Dir(__static_dir))

    // media file server
    __media_fileServer = http.FileServer(http.Dir(MEDIA_DIR))

    r := mux.NewRouter()

    r.PathPrefix("/").HandlerFunc(dispatcher)

    srv := &http.Server{
        Handler: r,
    }

    srv.Serve(listener)
}

func ListenHttp() {
    Header = "http://"
    // static file server
    __static_fileServer = http.FileServer(http.Dir(__static_dir))

    // media file server
    __media_fileServer = http.FileServer(http.Dir(MEDIA_DIR))

    r := mux.NewRouter()

    r.PathPrefix("/").HandlerFunc(__dispatcher)

    src := &http.Server{
        Addr:    CameraPort,
        Handler: r,
    }
    src.ListenAndServe()
}
