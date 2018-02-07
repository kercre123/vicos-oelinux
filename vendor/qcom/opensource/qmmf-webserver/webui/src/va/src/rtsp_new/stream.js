
import {Log} from 'bp_logger';
import {RTSPClientSM as RTSPClient}  from './client';
import {Url} from '../util/url';

export class RTSPStream {

    constructor(client, track) {
        this.state = null;
        this.client = client;
        this.track = track;
    }

    reset() {
        this.client.connection.backend.forgetRTPChannels();
        this.client = null;
        this.track = null;
    }

    start() {
        return this.sendSetup().then(this.sendPlay.bind(this));
    }

    stop() {
        return this.sendTeardown();
    }

    getSetupURL(track) {
        var sessionBlock = this.client.sdp.getSessionBlock();
        if (Url.isAbsolute(track.control)) {
            return track.control;
        } else if (Url.isAbsolute(`${sessionBlock.control}${track.control}`)) {
            return `${sessionBlock.control}${track.control}`;
        } else if (Url.isAbsolute(`${this.client.contentBase}${track.control}`)) {
            return `${this.client.contentBase}${track.control}`;
        }

        Log.error('Can\'t determine track URL from ' +
            'block.control:' + track.control + ', ' +
            'session.control:' + sessionBlock.control + ', and ' +
            'content-base:' + this.client.contentBase);
    }

    getControlURL() {
        let ctrl = this.client.sdp.getSessionBlock().control;
        if (Url.isAbsolute(ctrl)) {
            return ctrl;
        } else if (!ctrl || '*' === ctrl) {
            return this.client.contentBase;
        } else {
            return `${this.client.contentBase}${ctrl}`;
        }
    }

    sendRequest(_cmd, _params={}) {
        let params = {};
        if (this.session) {
            params['Session'] = this.session;
        }
        Object.assign(params, _params);
        return this.client.connection.sendRequest(_cmd, this.getControlURL(), params);
    }

    sendRequest2(_cmd, _params={}) {
        let params = {};
        if (this.session) {
            params['Session'] = this.session;
        }
        Object.assign(params, _params);
        return this.client.connection.sendRequest(_cmd, this.getSetupURL(this.track), params);
    }

    sendSetup() {
        this.state = RTSPClient.STATE_SETUP;
        let rtpChannel = this.client.interleaveChannelIndex;
        let interleavedChannels = this.client.interleaveChannelIndex++ + "-" + this.client.interleaveChannelIndex++;
        this.client.connection.backend.useRTPChannel(rtpChannel);
        return this.sendRequest2('SETUP', {
            'Transport': `RTP/AVP/TCP;unicast;interleaved=${interleavedChannels}`,
            'Date': new Date().toUTCString()
        }).then((_data)=>{
            let _array = _data.headers['session'].split(';');
            this.session = _array[0];
            this.client.timeout = _array.length == 1 ? 0 : Number(_array[1].split('=')[1]);
        });
    }

    sendPlay() {
        this.state = RTSPStream.STATE_PLAY;
        return this.sendRequest('PLAY').then((_data)=>{
            this.state = RTSPClient.STATE_PLAYING;
            return {track:this.track, data: _data};
        });
    }

    sendPause() {
        if (!this.client.supports("PAUSE")) {
            return;
        }
        this.state = RTSPClient.STATE_PAUSE;
        return this.sendRequest("PAUSE").then((_data)=>{
            this.state = RTSPClient.STATE_PAUSED;
        });
    }

    sendTeardown() {
      console.log('sendTeardown')
      console.log(this.state)
        if (this.state != RTSPClient.STATE_TEARDOWN) {
            this.client.connection.backend.forgetRTPChannels();
            this.state = RTSPClient.STATE_TEARDOWN;
            this.client.stopKeepAlive();
            return this.sendRequest("TEARDOWN").then(()=> {
              console.log('thenthen')
                Log.log('RTSPClient: STATE_TEARDOWN');
                this.client.connection.disconnect();
            });
        }
    }
}
