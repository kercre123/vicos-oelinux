#!/bin/bash -xe 

cd docker && docker build . -t vicos:latest 
export RSYNC_PASSWORD="bad_password"
container=$(docker run -d -p 10873:873 $1)
docker exec -it $container sh -c "chmod 777 /home/share"
sleep 5
rsync -avOP ./ rsync://build@localhost:10873/share
container="06c1f4568b2a"
sanity_file="/home/share/poky/meta/conf/sanity.conf"
docker exec -it $container bash -c "rm $sanity_file && touch $sanity_file "
docker exec -it $container bash -c "cd /home/share/poky && source build/conf/set_bb_env.sh && build-victor-robot-image"
rsync -avOP rsync://build@localhost:10873/share/_build ./_build
