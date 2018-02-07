
import { Log } from 'bp_logger';
import { WebSocketProxy } from '../util/websocket_proxy';
import { RTSPClientSM as RTSPClient } from './client';
import { EventEmitter } from 'bp_event';
import { RTSPMessage } from './message'
const MessageBuilder = new RTSPMessage(RTSPMessage.RTSP_1_0);

export class RTSPWebSocketBackend {
  constructor (host, port, auth, wsUrl) {
    this.rtp_handler = () => {}
    this.response_queue = [];

    this.host = host;
    this.port = port;
    this.auth = auth;
    this.wsUrl = wsUrl;

    this.eventSource = new EventEmitter();

    this.ready = this.connect();
    this.rtp_channels = new Set();
  }

  setEndpoint ({ host, port, auth }) {
    this.host = host;
    this.port = port;
    this.auth = auth;
  }

  reconnect () {
    return this.disconnect().then(()=> {
      return this.connect();
    });
  }

  connect () {
    this.rtpproxy = null;
    this.proxy = new WebSocketProxy(this.wsUrl, {
      host: this.host,
      port: this.port,
      auth: this.auth
    });
    this.proxy.set_message_handler((ev)=> {
      let item = this.response_queue.shift();
      item.resolve(ev.data);
    });
    this.proxy.set_disconnect_handler(()=> {
      if (this.rtpproxy) {
        this.rtpproxy.set_disconnect_handler(()=> {
        });
        this.rtpproxy.close();
      }
      this.eventSource.dispatchEvent('disconnected');
      setTimeout(()=> {
        this.ready = this.connect();
      }, 3000);
    });

    return this.proxy.connect("rtsp").then((id)=> {
      if (id == -1) {
        throw new Error("failed to connect");
      }
      this.rtpproxy = new WebSocketProxy(this.wsUrl, { sock_id: id });
      this.rtpproxy.set_message_handler((ev)=> {
        let channel = new DataView(ev.data).getUint8(1);

        if (this.rtp_channels.has(channel)) {
          this.rtp_handler({ packet: new Uint8Array(ev.data, 4), type: channel });
        }
      });
      this.rtpproxy.set_disconnect_handler(()=> {
        if (this.proxy) {
          this.proxy.close();
        }
      });
      return this.rtpproxy.connect('rtp').then(()=> {
        this.eventSource.dispatchEvent('connected');
      });
    });
  }

  disconnect () {
    console.log('disconnect')
    let promises = [this.proxy.close()];
    if (this.rtpproxy) {
      promises.push(this.rtpproxy.close());
    }
    return Promise.all(promises);
  }

  socket () {
    return this.proxy;
  }


  send (_data) {
    return new Promise((resolve, reject)=> {
      this.response_queue.push({ resolve, reject });
      this.proxy._send(_data);
    });
  }

  setRtpHandler (handler) {
    this.rtp_handler = handler;
  }

  useRTPChannel (channel) {
    this.rtp_channels.add(channel);
  }

  forgetRTPChannels () {
    this.rtp_channels.clear();
  }
}

export class RTPError {
  constructor (message, file, line) {
  }
}

export class RTSPConnection {

  constructor (_host, _port = 554, _uri, { login = '', password = '' }, backend, wsUrl) {
    let auth = login ? `${login}:${password}@` : '';
    this.url = `rtsp://${auth}${_host}:${_port}${_uri}`;
    this.requests = {};
    this.host = _host;
    this.port = _port;
    this.login = login;
    this.password = password;
    this.backend_constructor = backend;
    this.wsUrl = wsUrl
    this.connect();
  }

  get connected () {
    return this._backend;
  }

  connect () {
    this._backend = new this.backend_constructor(this.host, this.port, {
      login: this.login,
      password: this.password
    }, this.wsUrl);
    this.eventSource = this._backend.eventSource;
    this.cSeq = 0;
    return this._backend.ready;
  }

  disconnect () {
    this.cSeq = 0;
    this._backend.disconnect();
  }

  setEndpoint ({ host, port, urlpath, auth }) {
    this.url = urlpath;
    this._backend.setEndpoint({ host, port, auth });
  }

  reconnect () {
    this.cSeq = 0;
    return this._backend.reconnect();
  }

  get backend () {
    return this._backend;
  }

  parse (_data) {
    Log.debug(_data);
    let d = _data.split('\r\n\r\n');
    let parsed = MessageBuilder.parse(d[0]);
    let len = Number(parsed.headers['content-length']);
    if (len) {
      let d = _data.split('\r\n\r\n');
      parsed.body = d[1];
    } else {
      parsed.body = "";
    }
    return parsed
  }

  sendRequest (_cmd, _host, _params = {}, _payload = null) {
    this.cSeq++;
    Object.assign(_params, {
      CSeq: this.cSeq,
      'User-Agent': RTSPClient.USER_AGENT
    });
    if (_host != '*') {
    }
    return this.send(this.cSeq, MessageBuilder.build(_cmd, _host, _params, _payload));
  }

  send (_seq, _data) {
    return this._backend.ready.then(()=> {
      return this._backend.send(_data).then(this.parse.bind(this)).then((parsed)=> {
        if (parsed.code >= 300) {
          Log.error(parsed.statusLine);
          throw new Error(`RTSP error: ${parsed.code} ${parsed.message}`);
        }
        return parsed;
      });
    });
  }
}
