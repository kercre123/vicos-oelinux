#Web service implementation which exposes the QMMF framework API

#Testing
 You can use the following 'curl' commands from any host connected to the target device. Currently support is present for:
- Query status (This can be called in any state)
>curl 'http://device-ip:4000' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' -i

- Connect to QMMF recorder
>curl 'http://device-ip:4000/connect' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i

- FR rule add
>curl 'http://<device-ip>:4000/vamconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b58","event_type":8,"name":"face recognized","sensitivity":1,"status":1}], "zones" :  [{"id" : "d4668d05-faa3-zone-0001-fde575fa6b58","type" : 2,"points" : [{"x" : 100,"y" : 100}, {"x" : 100,"y" : 999}]}]}' -i

- FR rule remove
>curl 'http://<device-ip>:4000/vamremoveconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b58","event_type":8,"name":"face recognized","sensitivity":1,"status":1}]}' -i

- FD rules add
>curl 'http://<device-ip>:4000/vamconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b59","event_type":7,"name":"face recognized","sensitivity":1,"status":1}], "zones" :  [{"id" : "d4668d05-faa3-zone-0001-fde575fa6b59","type" : 2,"points" : [{"x" : 100,"y" : 100}, {"x" : 100,"y" : 999}]}]}' -i

- FD rule remove
>curl 'http://<device-ip>:4000/vamremoveconfig' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_config={"version":"1.0","id":"1f2d2853-f382-48d9-83a9-57e47e9c37ff","atomic_rules":[{"id":"d4668d05-faa3-4b2d-0001-fde575fa6b59","event_type":7,"name":"face recognized","sensitivity":1,"status":1}]}' -i

- Disenroll image to VAM
>curl 'http://device-ip:4000/vamdisenroll' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'vam_disenroll_event_type=8&vam_disenroll_id=0' -i

- Enroll image to VAM
>(echo -n "vam_enroll_object_type=3&vam_enroll_event_type=8&vam_enroll_format=7&vam_enroll_width=320&vam_enroll_height=400&vam_enroll_id=1&vam_enroll_name=testface&vam_enroll_image_id=1&vam_enroll_data="; cat <path-to-base64-url-encoded_grayscale_face>) | curl --data @- 'http://device-ip:4000/vamenroll' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' -i

- Execute VAM database command
>curl 'http://device-ip:4000/dbcommand' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'db_command=0&db_id=""&db_event_type=[1,5,7]&db_max_event_cnt=0&db_frame_ts=0&db_frame_ts1=0&db_page_index=0&db_index_range=[0,100,150,200]' -i

- Start camera
>curl 'http://device-ip:4000/startcamera' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'zsl_mode=0&zsl_queue_depth=10&zsl_width=1920&zsl_height=1080&framerate=30&flags=0&camera_id=0' -i

- Capture an image
>curl 'http://device-ip:4000/captureimage' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0&image_width=4000&image_height=3000&image_quality=95' -i

- Create a session
>curl 'http://device-ip:4000/createsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i

- Create a video track
>curl 'http://device-ip:4000/createvideotrack' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0&track_id=1&session_id=1&track_width=320&track_height=240&track_codec=1&bitrate=512000&framerate=30&track_output=0&low_power_mode=0' -i

- Start a session
>curl 'http://device-ip:4000/startsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1' -i

- Configure camera parameters
>curl 'http://<device-ip>:4000/setcameraparam' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0&ir_mode=on' -i

- Create user text overlay
>curl 'http://<device-ip>:4000/createoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_type=usertext&ov_position=topleft&ov_color=869007615&ov_user_text=test' -i

- Create datetime overlay
>curl 'http://<device-ip>:4000/createoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_type=datetime&ov_position=topright&ov_color=869007615&ov_date=mmddyyyy&ov_time=hhmmss_24hr' -i

- Create bounding box overlay
>curl 'http://<device-ip>:4000/createoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_type=boundingbox&ov_box_name=test&ov_start_x=100&ov_start_y=100&ov_width=100&ov_height=100&ov_color=869007615' -i

- Delete overlay
>curl 'http://<device-ip>:4000/deleteoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Remove overlay
>curl 'http://<device-ip>:4000/removeoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Set overlay
>curl 'http://<device-ip>:4000/setoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Get overlay
>curl 'http://<device-ip>:4000/getoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1' -i

- Update overlay
>curl 'http://<device-ip>:4000/updateoverlay' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'track_id=1&ov_id=1&ov_type=boundingbox&ov_box_name=slack&ov_start_x=100&ov_start_y=100&ov_width=100&ov_height=100&ov_color=869007615' -i

- Stop a session
>curl 'http://device-ip:4000/stopsession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1&flush=1' -i

- Delete a video track
>curl 'http://device-ip:4000/deletevideotrack' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1&track_id=1' -i

- Delete a session
>curl 'http://device-ip:4000/deletesession' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'session_id=1' -i

- Stop camera
>curl 'http://device-ip:4000/stopcamera' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data 'camera_id=0' -i

- Disconnect from QMMF recorder
>curl 'http://device-ip:4000/disconnect' -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' -H 'Accept: application/json, text/javascript, */*; q=0.01' -H 'X-Requested-With: XMLHttpRequest' -H 'Connection: keep-alive' --data '' -i

#API summary
'GET' requests to the server root will return a JSON response containing information about the server, camera capabilities, created streams, active RTSP urls etc. The JSON reply will have the following example structure:
```json
{
    "Tracks": [{
        "bitrate":10000000,
        "camera_id":0,
        "codec":1,
        "framerate":30,
        "height":1080,
        "output":0,
        "rtsp_url":"rtsp://<device-ip>:8900/live",
        "session_id":1,
        "track_id":1,
        "width":1920
    }],
    "AudioTracks":null,
    "Cameras":[{
        "camera_id":0,
        "flags":0,
        "framerate":30,
        "hdr_modes":" off on",
        "ir_modes":" off on",
        "nr_modes":" off fast high-quality minimal zsl",
        "zsl_height":1080,
        "zsl_mode":0,
        "zsl_q_depth":8,
        "zsl_width":1920
    }]
}
```

* __connect__ - Connects to the QMMF recorder service.
 * Arguments - None.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamconfig__ - Set VAM configuration. The configuration will get applied only in case VAM is active otherwise it will return error.
 * Arguments
   * vam_config - JSON array containing the VAM configuration. Check 'curl' example for reference.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamremoveconfig__ - Remove VAM configuration. The configuration will removed only in case VAM is active otherwise it will return error.
 * Arguments
   * vam_config - JSON array containing the VAM configuration. Check 'curl' example for reference.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamdisenroll__ - VAM image disenroll. This should get called only after a session with YUV track and VAM output gets started.
 * Arguments
   * vam_enroll_event_type - Enroll event type.
   * vam_enroll_id - This parameter could be the enrolled feature display name, or the enrolled image id. This is so because
                     for each enrolled feature there could be multiple images with unique ids. Passing the display name will
                     remove all enrolled images for that feature, while passing image id will remove just the unique id.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __vamenroll__ - VAM image enroll. This should get called only after a session with YUV track and VAM output gets started.
 * Arguments
   * vam_enroll_object_type - Enroll object type.
   * vam_enroll_event_type - Enroll event types. Format of the parameter is [int1,int2,int3,intN]
   * vam_enroll_format - Format of the enrolled image.
   * vam_enroll_width - Width of the enrolled image.
   * vam_enroll_height - Height of the enrolled image.
   * vam_enroll_id - Enroll id.
   * vam_enroll_name - Name of the enrolled frame.
   * vam_enroll_image_id - Id of the enrolled image.
   * vam_enroll_data - Base64 url encoded data of the enrolled image.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __dbcommand__ - Send various commands to QMMF webserver for the internal VAM database to process as queries.
 * Arguments
   * db_session - unique identifier of each session of VAM database. every time VAM is initialized it will create new session. This param can be empty string in which case all databases are queried.
   * db_session - the session for which the command will be executed. With this parameter earlier sessions can be viewed.
   * db_command - Integer value indicating the type of DB query to be performed. Available commands:
        GET_ALL_EVENT_IDS               = 0 - Input params: none. Returns a list of IDs in JSON format
        GET_ALL_FRAME_IDS               = 1 - Input params: none. Returns a list of IDs in JSON format
        GET_ALL_RULES_IDS               = 2 - Input params: none. Returns a list of IDs in JSON format
        GET_EVENT                       = 3 - Input params: 'db_id' - the event id. Returns the event in JSON format
        GET_METADATA_FOR_FRAME          = 4 - Input params: 'db_id' - the frame id. Returns a list of all objects metadata for that frame in JSON format
        GET_METADATA_FOR_EVENT          = 5 - Input params: 'db_id' - the event id. Returns a list of all objects metadata for that event in JSON format
        GET_SNAPSHOT                    = 6 - Input params: 'db_id' - the frame id. Returns the snapshot of the event in JPEG binary data
        GET_VIDEO                       = 7 - Input params: 'db_id' - the frame id. Returns the last few moments that caused the event in video format H264 binary format
        GET_RULE                        = 8 - Input params: 'db_id'. Returns the rule used for configuration in JSON format
        GET_EVENTS_FOR_TS_LIMIT         = 9 - Input params: 'db_frame_ts', 'db_frame_ts1`, 'db_event_type', 'db_max_event_cnt'. Returns the last events of the specified type in some time interval [ts, ts1] in JSON format.
        GET_LAST_EVENTS                 = 10- Input params: 'db_max_event_cnt', 'db_event_type'. Returns the last events of the specified type in JSON format. The input param can limit the retrieved events (0 for all).
        GET_EVENTS_FOR_FRAME            = 11- Input params: 'db_id', 'db_max_event_cnt' - the frame id. Returns all events for specific frame in JSON format.
        GET_EVENTS_FOR_RULE             = 12- Input params: 'db_id', 'db_max_event_cnt' - the rule id, 'db_max_event_cnt'. Returns all events for specific rule in JSON format. Second param can limit the retrieved events (0 for all).
        GET_EVENTS_FOR_RULE_TS_LIMIT    = 13- Input params: 'db_id' - the rule id, 'db_frame_ts', 'db_frame_ts1', 'db_max_event_cnt'. Returns all events for specific rule in some time interval [ts, ts1] in JSON format
        REMOVE_EVENT                    = 14- Input params: 'db_id'. No return data
        REMOVE_RULE                     = 15- Input params: 'db_id'. No return data
        GET_CURRENT_SESSION             = 16- Input params: none. Returns the current session of VAM database in JSON format
        GET_ALL_SESSIONS                = 17- Input params: none. Returns all sessions stored on the device in JSON format
        GET_SNAPSHOT_ID_FOR_EVENT       = 18- Input params: 'db_id' - the event id. Returns the snapshot id for the event in JSON format
        GET_SNAPSHOT_FOR_EVENT          = 19- Input params: 'db_id' - the event id. Returns the snapshot of the event in JPEG binary data
        GET_VIDEO_FOR_EVENT             = 20- Input params: 'db_id' - the event id. Returns the last few moments that caused the event in video format H264 binary format
        GET_THUMBNAIL                   = 21- Input params: 'db_id' - the frame id. Returns the thumbnail of the snapshot of the frame in JPEG binary data
        GET_THUMBNAIL_FOR_EVENT         = 22- Input params: 'db_id' - the event id. Returns the thumbnail of the snapshot of the event in JPEG binary data
        GET_EVENTS_TIMESTAMPS               = 23- Input params: none. Returns a list of all events' timestamps
        GET_EVENT_COUNT                     = 24- Input params: none. Returns the event count in the database
        SET_EVENT_PAGE_SIZE                 = 25- Input params: 'db_page_index' - the new number of events per page to be returned.
        GET_EVENT_PAGE_SIZE                 = 26- Input params: none. Returns the current number of events per page to be returned.
        GET_EVENT_PAGE                      = 27- Input params: 'db_page_index' - the page index from 0 to max_page. The max page is determined based on page size and all events. Returns a list of events in JSON format.
        GET_EVENTS_BY_INDEX_RANGE           = 28- Input params: 'db_page_index', 'db_index_range' - the range of event indices (starts from 0). Returns a list of events in JSON format.
        GET_EVENTS_BY_INDEX_RANGE_BEFORE_TS = 29- Input params: 'db_page_index', 'db_index_range', 'db_frame_ts' - the reference timestamp. Returns a list of events in JSON format.
        GET_EVENTS_BY_INDEX_RANGE_AFTER_TS  = 30- Input params: 'db_page_index', 'db_index_range', 'db_frame_ts'. Returns a list of events in JSON format.
        FRDB_GET_IMAGE_IDS                      = 100- Input params: none. Returns a list of the image ids for all enrolled images
        FRDB_GET_IMAGE_IDS_FOR_FEATURE          = 101- Input params: 'db_id'. For the provided feature id, returns a list of all image ids for that feature
        FRDB_GET_IMAGE_IDS_FOR_DISPLAY_NAME     = 102- Input params: 'db_id'. For the provided display name, returns a list of all image ids for features with that display name
        FRDB_GET_FEATURE_IDS                    = 103- Input params: none. Returns a list of the feature ids that were enrolles
        FRDB_GET_FEATURE_IDS_FOR_IMAGE          = 104- Input params: 'db_id'. For the provided image id, returns a list of all feature ids
        FRDB_GET_FEATURE_IDS_FOR_DISPLAY_NAME   = 105- Input params: 'db_id'. For the provided display name, returns a list of all feature ids
        FRDB_GET_ENROLL_INFO                    = 106- Input params: 'db_id'. Returns the enrollment information in JSON format for the requested image with image id 'db_id'
        FRDB_GET_ENROLL_IMAGE_DATA              = 107- Input params: 'db_id'. Returns the image data for the requested image with image id 'db_id'
        FRDB_REMOVE_ENROLLED_IMAGE              = 108- Input params: 'db_id'. Removes the enrollment information for the requested image with image id 'db_id'
   * db_id - An event/frame/rule/enroll ID parameter used by some of the DB queries
   * db_event_type - An event type parameter used by some of the DB queries
   * db_max_event_cnt - The maximum number of event entries to be returned by the database
   * db_frame_ts - A frame timestamp parameter used by some of the DB queries (start time for time intervals)
   * db_frame_ts1 - A frame timestamp parameter used by some of the DB queries (end time for time intervals)
   * db_page_index - A page index parameter. Can be used to set the page size as well
   * db_index_range - A list of index ranges for retrieval of event entries
 * Returns the command being executed and the execution status along with any data associated with the query.
* __startcamera__ - Starts a speficic camera.
 * Arguments
   * camera_id - Index of the camera that needs to be started.
    * zsl_mode - Enable ZSL. Possible values [0,1]
    * zsl_queue_depth - ZSL queue depth.
    * zsl_width - Width of the buffers used by ZSL.
    * zsl_height - Height of the buffers used by ZSL.
    * framerate - Framerate with which ZSL will run.
    * flags - Additional flags. Not used currently.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __captureimage__ - Takes a camera snapshot. The resulting Jpeg will get stored locally in the device '/data/misc/qmmf' folder.
 * Arguments
   * camera_id - Index of the camera that needs to capture an image.
   * image_width - Width of the captured image.
   * image_height - Height of the captured image.
   * image_quality - Jpeg compression. Maximum value is 100.
 * Returns JSON structure containing the Base64 encoded snapshot in the "Data" key and timestamp in "Timestamp". If an error occurs a status and error JSON structure will get sent back.
* __createsession__ - Creates a new recording session.
 * Arguments - None
 * In case of success it returns the session id in json structure. The field which contains the value will be called "SessionId" and the value will be non-negative. In case of error it will return a JSON status and error.
* __createaudiotrack__ - Creates a new audio track within a give recording session.
 * Arguments
   * sample_rate - Audio stream sample rate.
    * track_id - A unique positive track_id.
    * session_id - The session id in which the track will run. The session id should match a valid session created by a previous call to "createsession".
    * num_channels - The audio channel count.
    * bit_depth - Bits per sample.
    * track_codec - Codec to be used for this video track. Values should be within this range {0 (->PCM), 1 (->AAC), 2 (->AMR), 3 (->AMRWB)}.
    * bitrate - In case of codecs different from PCM.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deleteaudiotrack__ - Deletes a given audio track within a recording session.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
   * track_id - The id of track that needs to be removed.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __createvideotrack__ - Creates a new video track within a give recording session.
 * Arguments
   * camera_id - Index of the camera that should be used as input for the track.
    * track_id - A unique positive track_id.
    * session_id - The session id in which the track will run. The session id should match a valid session created by a previous call to "createsession".
    * track_width - The width of the video track.
    * track_height - The height of the video track.
    * track_codec - Codec to be used for this video track. Values should be within this range {0 (->HEVC), 1 (->AVC), 2 (->YUV), 3 (->RDI), 4 (->RAWIDEAL)}.
    * bitrate - In case of HEVC or AVC
    * framerate - Framerate with which the track will run.
    * low_power_mode - Enables low power mode (preview camera stream). Possible values {0 (Default disabled), 1 (Enabled)}.
    * track_output - The sink which will receive, process and potentially stream the track data. Values should be within this range {0 (->RTSP streaming), 1 (->Input in VAM), 2 (->MP4 muxing, stored in the local device '/data/misc/qmmf' directory), 3 (->3GP muxing, stored in the local device '/data/misc/qmmf' directory), 4 (->MpegTs muxing and streaming over RTSP), 5 (->RTMP streaming)}
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __startsession__ - Starts a particular recording session. All tracks added inside this session will start too.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __setcameraparam__ - Sets specific camera parameters. This specific request can be made after a session with at least one video track get started.
 * Arguments
   * camera_id - Index of the camera on which the parameters should get applied.
   *ir_mode - Infra red mode one of {off, on}. Please query camera status for supported values.
   *hdr_mode - HDR mode one of {off, on}. Please query camera status for supported values.
   *nr_mode - Noise reduction mode one of {off, fast, high-quality, minimal, zsl}. Please query camera status for supported values.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __createoverlay__ - Create video overlay. It can be called after initialization.
 * Arguments
   * ov_type - Type of overlay. It should be one of {datetime,usertext,staticimage,boundingbox,privacymask}
    * track_id - The track id to which this overlay will get attached.
    * ov_color - Text color value. Required for overlays of type {datetime,usertext,boundingbox,privacymask}
    * ov_position - Overlay position. It should be one of {topleft,topright,center,bottomleft,bottomright,none}. Required for overlays of type {datetime,usertext,staticimage}
    * ov_user_text - User defined text. Required for overlays of type {usertext}
    * ov_start_x - Overlay box starting X position. Required for overlays of type {boundingbox, privacymask}
    * ov_start_y - Overlay box starting Y position. Required for overlays of type {boundingbox, privacymask}
    * ov_width - Overlay width. Required for overlays of type {staticimage, boundingbox, privacymask}
    * ov_height - Overlay height. Required for overlays of type {staticimage, boundingbox, privacymask}
    * ov_image_location - Location of YUV image on device file system. Required for overlays of type {staticimage}
    * ov_box_name - Overlay bounding box title name. Required for overlays of type {boundingbox}
    * ov_date - Date format. It should be one of {yyyymmdd,mmddyyyy}. Required for overlays of type {datetime}
    * ov_time - Time format. It should be one of {hhmmss_24hr,hhmmss_ampm,hhmm_24hr,hhmm_ampm}. Required for overlays of type {datetime}
 * Returns JSON status and error. If error equals 'none', status will contain the Overlay ID, which can be used to reference the Overlay in subsequent overlay related operations.
* __updateoverlay__ - Update the parameters of already created overlay.
 * Arguments
   * ov_type - Type of overlay. It should be one of {datetime,usertext,staticimage,boundingbox,privacymask}
    * ov_id - Overlay id received during create.
    * track_id - The track id to which this overlay will get attached.
    * ov_color - Text color value. Required for overlays of type {datetime,usertext,boundingbox,privacymask}
    * ov_position - Overlay position. It should be one of {topleft,topright,center,bottomleft,bottomright,random,none}. Required for overlays of type {datetime,usertext,staticimage}
    * ov_user_text - User defined text. Required for overlays of type {usertext}
    * ov_start_x - Overlay box starting X position. Required for overlays of type {datetime, usertext, staticimage, boundingbox, privacymask} [this value will be used in case ov_position is random and will be ignored in other ov_position {datetime, usertext, staticimage} so keep it 0 for those cases]
    * ov_start_y - Overlay box starting Y position. Required for overlays of type {datetime, usertext, staticimage, boundingbox, privacymask} [this value will be used in case ov_position is random and will be ignored in other ov_position {datetime, usertext, staticimage} so keep it 0 for those cases]
    * ov_width - Overlay width. Required for overlays of type {datetime, usertext, staticimage, boundingbox, privacymask} [this value will be used in case ov_position is random and will be ignored in other ov_position {datetime, usertext} so keep it 0 for those cases]
    * ov_height - Overlay height. Required for overlays of type {datetime, usertext, staticimage, boundingbox, privacymask} [this value will be used in case ov_position is random and will be ignored in other ov_position {datetime, usertext} so keep it 0 for those cases]
    * ov_image_location - Location of YUV image on device file system. Required for overlays of type {staticimage}
    * ov_box_name - Overlay bounding box title name. Required for overlays of type {boundingbox}
    * ov_date - Date format. It should be one of {yyyymmdd,mmddyyyy}. Required for overlays of type {datetime}
    * ov_time - Time format. It should be one of {hhmmss_24hr,hhmmss_ampm,hhmm_24hr,hhmm_ampm}. Required for overlays of type {datetime}
	* ov_image_type - Image Type Format. Required for overlays of type {staticimage}.It should be {filetype}
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __getoverlay__ - Retrieve parameters of specific overlay.
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id to which this overlay will get attached.
 * On success returns a JSON object with all overlay parameters. For example a static image overlay could return:
```json
{
    "ov_color":869007615,
    "ov_date":"mmddyyyy",
    "ov_position":"topleft",
    "ov_time":"hhmmss_24hr",
    "ov_type":"datetime"
}
```
* __setoverlay__ - Set a particular overlay per track
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id to which this overlay will get attached.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __removeoverlay__ - Remove a particular overlay from give track
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id from which this overlay will get removed.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deleteoverlay__ - Deletes a specific overlay.
 * Arguments
   * ov_id - Overlay ID. This is the ID recevived as part of the status after a successfull overlay create
   * track_id - The track id from which this overlay will get delete.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __stopsession__ - Stops a particular recording session and all tracks contained within.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
    * flush - Valid values are in the range of {0 (->Don't discard any pending buffers in the pipeline), 1 (->Discard all buffers in the pipeline)}.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deletevideotrack__ - Deletes a given video track within a recording session.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
   * track_id - The id of track that needs to be removed.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __deletesession__ - Removes a particular recording session.
 * Arguments
   * session_id - Session id received during a previous call to "createsession".
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __stopcamera__ - Stops a given camera or cameras.
 * Arguments
   * camera_id - Index of the camera that needs to get stopped.
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.
* __disconnect__ - Disconnects from the QMMF recorder.
 * Arguments - none
 * Returns JSON status and error. If error is different than 'none', status will contain further information about the error.

