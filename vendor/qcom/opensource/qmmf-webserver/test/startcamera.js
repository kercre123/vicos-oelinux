/*
* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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
var system = require('system');

function startcamera(args) {
    var start_camera_url = "/startcamera";
    var start_camera_args = {
        operation: "POST",
        encoding: "utf8",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
            "X-Requested-With": "XMLHttpRequest",
            "Accept": "application/json, text/javascript, */*; q=0.01",
            "Connection": "keep-alive",
        },
        data: "camera_id=",
    };
    start_camera_args.data += args[2];
    start_camera_args.data += "&zsl_mode=" +args[3];
    start_camera_args.data += "&zsl_width=" +args[4];
    start_camera_args.data += "&zsl_height=" +args[5];
    start_camera_args.data += "&zsl_queue_depth=" +args[6];
    start_camera_args.data += "&framerate=" +args[7];
    start_camera_args.data += "&flags=" +args[8];
    var start_camera_page = require('webpage').create();
    var url = args[1] + start_camera_url;
    start_camera_page.open(url, start_camera_args, function(status) {
        var reply = JSON.parse(start_camera_page.plainText);
        console.log("request status: " + reply.Status);
        console.log("request error: " + reply.Error);
        phantom.exit();
    });
}

if (system.args.length < 9) {
    console.log("Usage: <server-url> <camera_id> <zsl_mode> <zsl_width> <zsl_height> <zsl_queue_depth> <framerate> <flags>");
    phantom.exit(1);
} else {
    startcamera(system.args);
}
