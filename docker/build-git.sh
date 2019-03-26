#!/usr/bin/env bash
set -x
set -e
git clone https://github.com/git/git
cd git
git checkout v2.1.2
make configure
./configure with-openssl
make prefix=/usr/local all
sudo make prefix=/usr/local install
