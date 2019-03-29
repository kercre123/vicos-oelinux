FROM ubuntu:xenial

RUN apt-get update && apt-get install -y software-properties-common

ADD . /scripts
RUN /scripts/install-deps.sh
RUN /scripts/build-git.sh

RUN sudo ln -sf /bin/bash /bin/sh
RUN adduser --disabled-password --gecos "" build

RUN sed -i '/^RSYNC_ENABLE=/c\RSYNC_ENABLE=inetd' /etc/default/rsync
COPY rsync/rsync /etc/xinetd.d/rsync
COPY rsync/rsyncd.conf /etc/rsyncd.conf
COPY rsync/rsyncd.secrets /etc/rsyncd.secrets

RUN chmod 600 /etc/rsyncd.secrets
RUN mkdir /home/share

CMD /etc/init.d/xinetd start && tail -f /dev/null