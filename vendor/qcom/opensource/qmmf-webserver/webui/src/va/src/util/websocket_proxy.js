
import { Log } from 'bp_logger';

export class WebSocketProxy {
  constructor (wsurl, data) {
    this.url = wsurl;
    this.data = data;
    this.message_handler = ()=> {
    }
    this.disconnect_handler = ()=> {
    }
  }

  set_message_handler (handler) {
    this.message_handler = handler;
  }

  set_disconnect_handler (handler) {
    this.disconnect_handler = handler;
  }

  close () {
    console.log('close')
    return new Promise((resolve)=> {
      this.sock.onclose = ()=> {
        this.sock = undefined;
        resolve();
      };
      this.sock.close();
    });
  }

  connect (protocol) {
    return new Promise((resolve, reject)=> {
      this.sock = new WebSocket(this.url, protocol);
      this.protocol = protocol;
      this.sock.binaryType = 'arraybuffer';
      this.connected = false;
      this.sock.onopen = ()=> {
        if (protocol == "rtsp") {
          this._send(`WSP 1.0 INIT\r\nhost ${this.data.host}\r\nport ${this.data.port}\r\n\r\n`);
        } else if (protocol == "rtp") {
          this._send(`WSP 1.0 INIT\r\nRTSP ${this.data.sock_id}\r\n\r\n`);
        }
      };
      this.sock.onmessage = (ev)=> {
        if (ev.data.startsWith('INIT')) {
          this.sock.onmessage = (e)=> {
            this.message_handler(e);
          };
          resolve(ev.data.substr(4).trim());
        } else {
          reject();
        }
      };
      this.sock.onerror = (e)=> {
        Log.error(`[${this.protocol}] ${e.type}`);
        this.sock.close();
      };
      this.sock.onclose = (e)=> {
        Log.error(`[${this.protocol}] ${e.type}. code: ${e.code}`);
        this.disconnect_handler();
      };
    });
  }

  _send (data) {
    try {
      this.sock.send(data);
    } catch (e) {
      throw(e);
    }
  }
}
