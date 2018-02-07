
export class Url {
    static parse(url) {
        var ret = {};

        var regex = /^([^:]+):\/\/([^\/]+)(.*)$/;
        var result = regex.exec(url);

        ret.full = url;
        ret.protocol = result[1];
        ret.urlpath = result[3];

        var parts = ret.urlpath.split('/');
        ret.basename = parts.pop().split(/\?|#/)[0];
        ret.basepath = parts.join('/');

        var loginSplit = result[2].split('@');
        var hostport = loginSplit[0].split(':');
        var userpass = [ null, null ];
        if (loginSplit.length === 2) {
            userpass = loginSplit[0].split(':');
            hostport = loginSplit[1].split(':');
        }

        ret.user = userpass[0];
        ret.pass = userpass[1];
        ret.host = hostport[0];

        ret.port = (null == hostport[1]) ? Url.protocolDefaultPort(ret.protocol) : hostport[1];
        ret.portDefined = (null != hostport[1]);

        return ret;
    }

    static isAbsolute(url) {
        return /^[^:]+:\/\//.test(url);
    }

    static protocolDefaultPort(protocol) {
        switch (protocol) {
            case 'rtsp': return 554;
        }

        return 0;
    }
}