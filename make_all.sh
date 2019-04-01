#!/bin/bash -xe 

function cleanup {
    if [ -n "$container" ]; then
        docker container kill $container
    else
        echo "Exiting..."
    fi
}
trap cleanup EXIT

function docker_exec {
    docker exec -it $container bash -c "$1"
}

function build_vicos {
    docker_exec "chown -R build /home/share"
    rsync -avOP ./ rsync://build@localhost:10873/share/vicos
    sanity="/home/share/vicos/poky/meta/conf/sanity.conf"
    docker_exec "rm $sanity && touch $sanity"
    docker_exec "cd /home/share/vicos/poky && source build/conf/set_bb_env.sh && build-victor-robot-image"
    rsync -avOP rsync://build@localhost:10873/share/_build ./_build
}

function setup_environment {
    export RSYNC_PASSWORD="bad_password"
    container=$(docker run -d -p 10873:873 $1)
    echo "Sleeping for 5 seconds to ensure xinetd is running..."
    sleep 5
}

#cd docker && docker build . -t vicos:latest 
if [[ $# -eq 0 ]]; then
    echo "Please provide docker image name"
    exit 1
fi
setup_environment $1
build_vicos