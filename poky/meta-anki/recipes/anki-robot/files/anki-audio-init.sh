#!/bin/sh

# Alsa: JMR/BRC
for i in `seq 1 10`;
do
    tinymix set "PRI_MI2S_RX Audio Mixer MultiMedia1" 1
    if [ $? -eq 0 ]; then
        break;
    fi
    sleep 0.5;
done
    
set -e
tinymix set "RX3 MIX1 INP1" "RX1"
tinymix set "SPK" 1
tinymix set "RX3 Digital Volume" 80
