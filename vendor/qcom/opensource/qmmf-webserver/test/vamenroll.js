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

function configurevam(args) {
    var vamenroll_url = "/vamenroll";
    var vamenroll_args = {
        operation: "POST",
        encoding: "utf8",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
            "X-Requested-With": "XMLHttpRequest",
            "Accept": "application/json, text/javascript, */*; q=0.01",
            "Connection": "keep-alive",
        },
        data: "vam_enroll_object_type="
    };
    vamenroll_args.data += args[2];
    vamenroll_args.data += "&vam_enroll_event_type=" +args[3];
    vamenroll_args.data += "&vam_enroll_format=" +args[4];
    vamenroll_args.data += "&vam_enroll_width=" +args[5];
    vamenroll_args.data += "&vam_enroll_height=" +args[6];
    vamenroll_args.data += "&vam_enroll_id=" +args[7];
    vamenroll_args.data += "&vam_enroll_name=" +args[8];
    vamenroll_args.data += "&vam_enroll_image_id=" +args[9];
    var fs = require('fs');
    var base64_data = fs.read(args[10]);
    var url_encoded_data = encodeURIComponent(base64_data);
    vamenroll_args.data += "&vam_enroll_data=" + url_encoded_data;

    var vamenroll_page = require('webpage').create();
    var url = args[1] + vamenroll_url;
    vamenroll_page.open(url, vamenroll_args, function(status) {
        var reply = JSON.parse(vamenroll_page.plainText);
        console.log("request status: " + reply.Status);
        console.log("request error: " + reply.Error);
        phantom.exit();
    });
}

if (system.args.length < 11) {
    console.log("Usage: <server-url> <object_type> <event_type> <format> <width> <height> <id> <name> <image_id> <base64_path>");
    phantom.exit(1);
} else {
    configurevam(system.args);
}
