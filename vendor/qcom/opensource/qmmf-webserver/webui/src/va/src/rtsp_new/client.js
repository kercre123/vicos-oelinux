
'use strict'

import { StateMachine } from 'bp_statemachine'
import { EventEmitter } from 'bp_event';
import { SDPParser } from './sdp';
import { RTSPStream } from './stream';
import { Log } from 'bp_logger';
import { RTP } from './rtp';

export class RTSPClientSM extends StateMachine {

  static USER_AGENT = 'SFRtsp 0.2';
  static STATE_INITIAL = 1 << 0;
  static STATE_OPTIONS = 1 << 1;
  static STATE_DESCRIBE = 1 << 2;
  static STATE_SETUP = 1 << 3;
  static STATE_STREAMS = 1 << 4;
  static STATE_TEARDOWN = 1 << 5;

  constructor (connection) {
    super()
    this.connection = connection
    this.eventSource = new EventEmitter();
    this.space = 0
    this.keepaliveInterval = null;

    this.reset();

    this.addState(RTSPClientSM.STATE_INITIAL, {}).addState(RTSPClientSM.STATE_OPTIONS, {
      activate: this.sendOptions,
      finishTransition: this.onOptions
    }).addState(RTSPClientSM.STATE_DESCRIBE, {
      activate: this.sendDescribe,
      finishTransition: this.onDescribe
    }).addState(RTSPClientSM.STATE_SETUP, {
      activate: this.sendSetup,
      finishTransition: this.onSetup
    }).addState(RTSPClientSM.STATE_STREAMS, {}).addState(RTSPClientSM.STATE_TEARDOWN, {
      activate: ()=> {
        this.started = false;
        return this.streams['application'].sendTeardown();
      },
      finishTransition: ()=> {
        return this.transitionTo(RTSPClientSM.STATE_INITIAL);
      }
    }).addTransition(RTSPClientSM.STATE_INITIAL, RTSPClientSM.STATE_OPTIONS)
      .addTransition(RTSPClientSM.STATE_OPTIONS, RTSPClientSM.STATE_DESCRIBE)
      .addTransition(RTSPClientSM.STATE_DESCRIBE, RTSPClientSM.STATE_SETUP)
      .addTransition(RTSPClientSM.STATE_SETUP, RTSPClientSM.STATE_STREAMS)
      .addTransition(RTSPClientSM.STATE_STREAMS, RTSPClientSM.STATE_TEARDOWN)
      .addTransition(RTSPClientSM.STATE_TEARDOWN, RTSPClientSM.STATE_INITIAL)
      .addTransition(RTSPClientSM.STATE_STREAMS, RTSPClientSM.STATE_TEARDOWN)
      .addTransition(RTSPClientSM.STATE_SETUP, RTSPClientSM.STATE_TEARDOWN)
      .addTransition(RTSPClientSM.STATE_DESCRIBE, RTSPClientSM.STATE_TEARDOWN)
      .addTransition(RTSPClientSM.STATE_OPTIONS, RTSPClientSM.STATE_TEARDOWN);

    this.transitionTo(RTSPClientSM.STATE_INITIAL);
  }

  reset () {
    for (let stream in this.streams) {
      if (this.streams.hasOwnProperty(stream)) {
        this.streams[stream].reset();
      }
    }
    this.streams = {};
    this.contentBase = "";
    this.state = RTSPClientSM.STATE_INITIAL;
    this.sdp = null;
    this.interleaveChannelIndex = 0;
    this.session = null;
    this.vtrack_idx = -1;
    this.atrack_idx = -1;
  }

  sendOptions () {
    this.reset();
    this.started = true;
    this.connection.cSeq = 0;
    return this.connection.sendRequest('OPTIONS', '*', {});
  }

  onOptions (data) {
    this.methods = data.headers['public'].split(',').map((e)=>e.trim());
    this.transitionTo(RTSPClientSM.STATE_DESCRIBE);
  }

  sendDescribe () {
    return this.connection.sendRequest('DESCRIBE', this.connection.url, {
      'Accept': 'application/sdp'
    }).then((data)=> {
      this.sdp = new SDPParser();
      return this.sdp.parse(data.body).catch(()=> {
        throw new Error("Failed to parse SDP");
      }).then(()=> {
        return data;
      });
    });
  }

  onDescribe (data) {
    this.contentBase = data.headers['content-base'];
    this.tracks = this.sdp.getMediaBlockList();
    Log.log('SDP contained ' + this.tracks.length + ' track(s). Calling SETUP for each.');

    if (data.headers['session']) {
      this.session = data.headers['session'];
    }

    if (!this.tracks.length) {
      throw new Error("No tracks in SDP");
    }

    this.transitionTo(RTSPClientSM.STATE_SETUP);
  }

  sendSetup () {
    if (this.tracks.length > 1) {
      let track_type = this.tracks[0];
      let track = this.sdp.getMediaBlock(track_type);
      this.streams[track_type] = new RTSPStream(this, track);
      this.streams[track_type].sendSetup().then(()=> {
        let track_type = this.tracks[1];
        let track = this.sdp.getMediaBlock(track_type);
        this.streams[track_type] = new RTSPStream(this, track);
        this.streams[track_type].session = this.streams['application'].session;
        let playPromise = this.streams[track_type].start();
        playPromise.then(({ track, data })=> {
          var idx;
          let timeOffset = 0;
          let rtp_infos = data.headers["rtp-info"].split(',');
          for (idx = 0; idx < 2; idx++) {
            let rtp_info = rtp_infos[idx];
            let _array = rtp_info.split('=');
            timeOffset = Number(_array[_array.length - 1]);
          }

          this.eventSource.dispatchEvent('playing');
          this.startKeepAlive(this.timeout);
        });
      });
    } else {
      let track_type = this.tracks[0];
      let track = this.sdp.getMediaBlock(track_type);
      this.streams[track_type] = new RTSPStream(this, track);
      let playPromise = this.streams[track_type].start();
      playPromise.then(({ track, data })=> {
        let timeOffset = 0;
        try {
          let rtp_info = data.headers["rtp-info"].split(';');
          timeOffset = Number(rtp_info[rtp_info.length - 1].split("=")[1]);
        } catch (e) {
          timeOffset = new Date().getTime();
        }

        this.eventSource.dispatchEvent('playing');
        this.startKeepAlive(this.timeout);
      });
    }

    this.connection.backend.setRtpHandler(this.onRTP.bind(this));

    return new Promise((resolve, reject)=> {
      this.eventSource.addEventListener('playing', resolve);
    });
  }

  onSetup () {
    this.transitionTo(RTSPClientSM.STATE_STREAMS);
  }

  startKeepAlive (timeo) {
    if (timeo == 0) {
      console.log('timeout = 0');
      return;
    }
    this.keepaliveInterval = setInterval(()=> {
      this.connection.sendRequest('GET_PARAMETER', this.connection.url, {
        'Session': this.streams['application'].session
      });
    }, (timeo - 5) * 1000);
  }

  stopKeepAlive () {
    clearInterval(this.keepaliveInterval);
  }

  onRTP (_data) {
    const rtp = new RTP(_data.packet, this.sdp);
    if (rtp.data.length === 0 && this.space <=8) {
      this.space++
    } else {
      this.space = 0
      const string = new TextDecoder('utf-8').decode(rtp.data)
      this.eventSource.dispatchEvent('sending', string)
    }
    if (this.space >= 8) {
      this.space = 0
      return this.eventSource.dispatchEvent('clear')
    }

  }
}
