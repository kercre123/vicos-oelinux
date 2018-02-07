
export class RTSPMessage {
    static RTSP_1_0 = "RTSP/1.0";

    constructor(_rtsp_version) {
        this.version = _rtsp_version;
    }

    build(_cmd, _host, _params={}, _payload=null) {
        let requestString = `${_cmd} ${_host} ${this.version}\r\n`;
        for (let param in _params) {
            requestString+=`${param}: ${_params[param]}\r\n`
        }
        if (_payload) {
            requestString+=`Content-Length: ${_payload.length}\r\n`
        }
        requestString+='\r\n';
        if (_payload) {
            requestString+=_payload;
        }
        return requestString;
    }

    parse(_data) {
        let lines = _data.split('\r\n');
        let parsed = {
            headers:{},
            body:null,
            code: 0,
            statusLine: ''
        };

        let match;
        [match, parsed.code, parsed.statusLine] = lines[0].match(new RegExp(`${this.version}[ ]+([0-9]{3})[ ]+(.*)`));
        parsed.code = Number(parsed.code);
        let lineIdx = 1;

        while (lines[lineIdx]) {
            let [k,v] = lines[lineIdx].split(/:(.+)/);
            parsed.headers[k.toLowerCase()] = v.trim();
            lineIdx++;
        }

        parsed.body = lines.slice(lineIdx).join('\n\r');

        return parsed;
    }

}