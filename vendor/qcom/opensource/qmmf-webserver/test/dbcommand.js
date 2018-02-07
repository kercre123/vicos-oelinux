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
var system = require('system');

function dbcommand(args) {
    var cmd_image_url = "/dbcommand";
    var cmd_image_args = {
        operation: "POST",
        encoding: "utf8",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
            "X-Requested-With": "XMLHttpRequest",
            "Accept": "application/json, text/javascript, */*; q=0.01",
            "Connection": "keep-alive",
        },
        data: "",
    };


    cmd_image_args.data += args[3];
    var cmd_image_page = require('webpage').create();
    var url = args[1] + cmd_image_url;
    cmd_image_page.open(url, cmd_image_args, function(status) {
        var reply = JSON.parse(cmd_image_page.plainText);
        console.log("request status: " + reply.Status);
        console.log("request error: " + reply.Error);
        if (reply.Status == 0) {
            var fs = require('fs');
            var stream = fs.open(args[2], 'w');
            stream.write(reply.Data);
            stream.close();
        }
        phantom.exit();
    });
}

if (system.args.length < 4) {
    console.log("Usage: <server-url> <output_file> <db_session=&db_command=0&db_id=&db_event_type=[1,3,7]&db_max_event_cnt=10&db_frame_ts=0&db_frame_ts1=0&db_page_index=0&db_index_range=[0, 10]");
    phantom.exit(1);
} else {
    dbcommand(system.args);
}
