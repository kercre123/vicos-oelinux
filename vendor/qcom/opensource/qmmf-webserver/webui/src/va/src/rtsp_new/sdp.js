
import {Log} from 'bp_logger';

export class SDPParser {
    constructor(){
        this.version = -1;
        this.origin = null;
        this.sessionName = null;
        this.timing = null;
        this.sessionBlock = {};
        this.media = {};
        this.tracks = {};
    }

    parse(content) {
        Log.debug(content);

        return new Promise((resolve, reject)=>{
            var dataString = content;
            var success = true;
            var currentMediaBlock = this.sessionBlock;

            for (let line of dataString.split("\n")) {
                line = line.replace(/\r/, '');
                if (0 === line.length) {
                    continue;
                }

                switch (line.charAt(0)) {
                    case 'v':
                        if (-1 !== this.version) {
                            Log.log('Version present multiple times in SDP');
                            reject();
                            return false;
                        }
                        success = success && this._parseVersion(line);
                        break;

                    case 'o':
                        if (null !== this.origin) {
                            Log.log('Origin present multiple times in SDP');
                            reject();
                            return false;
                        }
                        success = success && this._parseOrigin(line);
                        break;

                    case 's':
                        if (null !== this.sessionName) {
                            Log.log('Session Name present multiple times in SDP');
                            reject();
                            return false;
                        }
                        success = success && this._parseSessionName(line);
                        break;

                    case 't':
                        if (null !== this.timing) {
                            Log.log('Timing present multiple times in SDP');
                            reject();
                            return false;
                        }
                        success = success && this._parseTiming(line);
                        break;

                    case 'm':
                        if (null !== currentMediaBlock && this.sessionBlock !== currentMediaBlock) {
                            this.media[currentMediaBlock.type] = currentMediaBlock;
                        }

                        currentMediaBlock = {};
                        currentMediaBlock.rtpmap = {};
                        this._parseMediaDescription(line, currentMediaBlock);
                        break;

                    case 'a':
                        SDPParser._parseAttribute(line, currentMediaBlock);
                        break;

                    default:
                        Log.log('Ignored unknown SDP directive: ' + line);
                        break;
                }

                if (!success) {
                    reject();
                    return;
                }
            }

            this.media[currentMediaBlock.type] = currentMediaBlock;

            success?resolve():reject();
        });
    }

    _parseVersion(line) {
        var matches = line.match(/^v=([0-9]+)$/);
        if (0 === matches.length) {
            Log.log('\'v=\' (Version) formatted incorrectly: ' + line);
            return false;
        }

        this.version = matches[1];
        if (0 != this.version) {
            Log.log('Unsupported SDP version:' + this.version);
            return false;
        }

        return true;
    }

    _parseOrigin(line) {
        var matches = line.match(/^o=([^ ]+) ([0-9]+) ([0-9]+) (IN) (IP4|IP6) ([^ ]+)$/);
        if (0 === matches.length) {
            Log.log('\'o=\' (Origin) formatted incorrectly: ' + line);
            return false;
        }

        this.origin = {};
        this.origin.username       = matches[1];
        this.origin.sessionid      = matches[2];
        this.origin.sessionversion = matches[3];
        this.origin.nettype        = matches[4];
        this.origin.addresstype    = matches[5];
        this.origin.unicastaddress = matches[6];

        return true;
    }

    _parseSessionName(line) {
        var matches = line.match(/^s=([^\r\n]+)$/);
        if (0 === matches.length) {
            Log.log('\'s=\' (Session Name) formatted incorrectly: ' + line);
            return false;
        }

        this.sessionName = matches[1];

        return true;
    }

    _parseTiming(line) {
        var matches = line.match(/^t=([0-9]+) ([0-9]+)$/);
        if (0 === matches.length) {
            Log.log('\'t=\' (Timing) formatted incorrectly: ' + line);
            return false;
        }

        this.timing = {};
        this.timing.start = matches[1];
        this.timing.stop  = matches[2];

        return true;
    }

    _parseMediaDescription(line, media) {
        var matches = line.match(/^m=([^ ]+) ([^ ]+) ([^ ]+)[ ]/);
        if (0 === matches.length) {
            Log.log('\'m=\' (Media) formatted incorrectly: ' + line);
            return false;
        }

        media.type  = matches[1];
        media.port  = matches[2];
        media.proto = matches[3];
        media.fmt   = line.substr(matches[0].length).split(' ').map(function(fmt, index, array) {
            return parseInt(fmt);
        });

        return true;
    }

    static _parseAttribute(line, media) {
        if (null === media) {
            return true;
        }

        var matches;
        var separator = line.indexOf(':');
        var attribute = line.substr(0, (-1 === separator) ? 0x7FFFFFFF : separator);

        switch (attribute) {
            case 'a=recvonly':
            case 'a=sendrecv':
            case 'a=sendonly':
            case 'a=inactive':
                media.mode = line.substr('a='.length);
                break;

            case 'a=control':
                media.control = line.substr('a=control:'.length);
                break;

            case 'a=rtpmap':
                matches = line.match(/^a=rtpmap:(\d+) (.*)$/);
                if (null === matches) {
                    Log.log('Could not parse \'rtpmap\' of \'a=\'');
                    return false;
                }

                var payload = parseInt(matches[1]);
                media.rtpmap[payload] = {};

                var attrs = matches[2].split('/');
                media.rtpmap[payload].name  = attrs[0];
                media.rtpmap[payload].clock = attrs[1];
                if (undefined !== attrs[2]) {
                    media.rtpmap[payload].encparams = attrs[2];
                }

                break;

            case 'a=fmtp':
                matches = line.match(/^a=fmtp:(\d+) (.*)$/);
                if (0 === matches.length) {
                    Log.log('Could not parse \'fmtp\'  of \'a=\'');
                    return false;
                }

                media.fmtp = {};
                for (var param of matches[2].split(';')) {
                    var idx = param.indexOf('=');
                    media.fmtp[param.substr(0, idx).toLowerCase().trim()] = param.substr(idx + 1).trim();
                }
                break;
        }

        return true;
    }

    getSessionBlock() {
        return this.sessionBlock;
    }

    hasMedia(mediaType) {
        return this.media[mediaType] != undefined;
    }

    getMediaBlock(mediaType) {
        return this.media[mediaType];
    }

    getMediaBlockByPayloadType(pt) {
        for (var m in this.media) {
            if (-1 !== this.media[m].fmt.indexOf(pt)) {
                return this.media[m];
            }
        }

        Log.error(`failed to find media with payload type ${pt}`);
        return null;
    }

    getMediaBlockList() {
        var res = [];
        for (var m in this.media) {
            res.push(m);
        }

        return res;
    }
}