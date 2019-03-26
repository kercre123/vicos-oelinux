#!/bin/bash 

sudo ln -sf /bin/bash /bin/sh
cd /vicos/poky
rm meta/conf/sanity.conf
touch meta/conf/sanity.conf
source build/conf/set_bb_env.sh
