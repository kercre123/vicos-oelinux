#!/bin/bash

# Copyright (c) 2016, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#     * Neither the name of The Linux Foundation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

adb forward tcp:4000 tcp:4000
adb forward tcp:8899 tcp:8899
adb forward tcp:8900 tcp:8900

CAMERA_ID=0
[[ "$1" -eq "1" ]] && CAMERA_ID=1

printf "\nConnect to QMMF recorder\n"
curl 'http://localhost:4000/connect' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i
printf "\n"

printf "\nStart camera\n"
curl 'http://localhost:4000/startcamera' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data "zsl_mode=0&zsl_queue_depth=8&zsl_width=1920&zsl_height=1080&framerate=30&flags=0&camera_id=$CAMERA_ID" -i
printf "\n"

printf "\nCreate a session\n"
curl 'http://localhost:4000/createsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i
printf "\n"

printf "\nCreate a video track\n"
curl 'http://localhost:4000/createvideotrack' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data "camera_id=$CAMERA_ID&track_id=1&session_id=1&track_width=1920&track_height=1080&track_codec=1&bitrate=10000000&framerate=30.0&track_output=0&low_power_mode=0" -i
printf "\n"

printf "\nStart a session\n"
curl 'http://localhost:4000/startsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1' -i
printf "\n"
