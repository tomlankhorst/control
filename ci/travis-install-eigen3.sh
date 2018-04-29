#!/usr/bin/env sh
# Installs eigen to $HOME/local

set -ex

wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
tar -xvf 3.3.4.tar.bz2
rm 3.3.4.tar.bz2
cd eigen-eigen-5a0156e40feb
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/local
make install
cd ../../
rm -r eigen-eigen-5a0156e40feb
