#!/usr/bin/env bash

set -e
set -u

function usage()
{
    echo "usage: update-os [-h|lkg|latest|version|url]"
    echo "-h                   This message"
    echo "lkg                  Update to Last Known Good OS (default) via full OTA"
    echo "latest               Update to latest OS via full OTA"
    echo "version              Like 1.2.1.2210 - Update to a specific version via full OTA"
    echo "delta-latest         Update to latest OS via delta OTA (may not work)"
    echo "url                  Give a full url like http://mylaptop:5555/os.ota"
    echo ""
    echo "It will take 10 seconds or so for the download to get going and"
    echo "then you should see some progress indication.  After the install"
    echo "is done, the robot will automatically reboot."
    echo ""
    exit 0
}

trap ctrl_c INT

function ctrl_c() {
    echo -e "\n\nStopping OS update and exiting..."
    systemctl -q stop update-engine
    exit 1
}

BASE_URL=`egrep UPDATE_ENGINE_BASE_URL= /anki/etc/update-engine.env | awk -F= '{print $NF;}'`
BASE_URL_LATEST=`egrep UPDATE_ENGINE_BASE_URL_LATEST /anki/etc/update-engine.env | awk -F= '{print $NF;}'`
if [ -z "${BASE_URL_LATEST}" ]; then
    BASE_URL_LATEST="${BASE_URL}"
fi
URL="${BASE_URL}full/lkg.ota"
if [ $# -gt 0 ]; then
    case "$1" in
	-h)
	    usage
	    ;;
	latest)
	    URL="${BASE_URL}full/latest.ota"
	    ;;
        delta-latest)
            URL="${BASE_URL_LATEST}diff/`getprop ro.anki.version | tr -d '[a-z]'`.ota"
            ;;
	lkg)
	    ;;
	[0-9].[0-9].[0-9].[0-9]*)
	    URL="${BASE_URL}full/$1.ota"
	    ;;
	*)
	    URL=$1
	    ;;
    esac
fi


systemctl -q stop update-engine

echo "Current OS Version: `getprop ro.anki.version`"

mkdir -p /run/vic-switchboard

echo "UPDATE_ENGINE_ENABLED=True" > /run/vic-switchboard/update-engine.env
echo "UPDATE_ENGINE_MAX_SLEEP=1" >> /run/vic-switchboard/update-engine.env
echo "UPDATE_ENGINE_ALLOW_DOWNGRADE=True" >> /run/vic-switchboard/update-engine.env
echo "UPDATE_ENGINE_URL=$URL" >> /run/vic-switchboard/update-engine.env
echo "UPDATE_ENGINE_DEBUG=True" >> /run/vic-switchboard/update-engine.env

chown -R net:anki /run/vic-switchboard

systemctl restart update-engine

echo -e "Downloading OS update from:\n$URL"

echo -e -n "\r."
DOTS=1
UPDATE_VERSION=""
while [[ ! -f /run/update-engine/done ]] ; do
    sleep 3
    if [ -z "${UPDATE_VERSION}" -a -f /run/update-engine/manifest.ini ]; then
	UPDATE_VERSION=`egrep update_version /run/update-engine/manifest.ini | awk -F= '{print $NF;}'`
    fi
    if [ -f /run/update-engine/progress -a -f /run/update-engine/expected-size ] ; then
	PROGRESS=`cat /run/update-engine/progress`
	EXPECTED=`cat /run/update-engine/expected-size`
        if [ -n "$PROGRESS" -a -n "$EXPECTED" ]; then
	    PCT=$(( 100 * $PROGRESS / $EXPECTED ))
	    echo -e -n "\rUpdating to ${UPDATE_VERSION} ( ${PCT}% )"
        fi
    else
	for ((i=0;i<$DOTS;i++)); do
	    echo -n "."
	done
	DOTS=$((DOTS+1))
    fi
    if [ -f /run/update-engine/error ]; then
	ERRORMSG=`cat /run/update-engine/error`
	if [ "$ERRORMSG" != "Unclean exit" ]; then
	    echo "Error updating OS . $ERRORMSG"
	    exit 1
	fi
    fi

done

echo -e "\n\nRebooting....."

sleep 2
sync
reboot & exit
