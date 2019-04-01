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
RUN chmod 777 /home/share

RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt-get update && apt-get install -y nodejs

RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt-get install git-lfs

RUN chmod 0755 /usr/local/bin
RUN wget https://github.com/ninja-build/ninja/releases/download/v1.8.2/ninja-linux.zip
RUN unzip ninja-linux.zip -d /usr/local/bin/
RUN update-alternatives --install /usr/bin/ninja ninja /usr/local/bin/ninja 1 --force
RUN ln -sf /usr/bin/nodejs /usr/local/bin/node

CMD /etc/init.d/xinetd start && tail -f /dev/null