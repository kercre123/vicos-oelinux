FROM ubuntu:xenial

RUN apt-get update && apt-get install -y software-properties-common

ADD . /scripts
RUN /scripts/install-deps.sh
RUN /scripts/build-git.sh

RUN sudo ln -sf /bin/bash /bin/sh
RUN adduser --disabled-password --gecos "" build