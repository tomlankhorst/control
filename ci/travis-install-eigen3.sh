#!/usr/bin/env sh
# Installs eigen to $HOME/local

set -ex

DIR=$HOME/local/include/eigen3
if [ -d $DIR ]; then
  echo "Dir $DIR exists. Skipping."
  exit 0
else
  echo "Dir $DIR does not exist. Building."
fi

wget -O eigen.tar.bz2 http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
tar -xvf eigen.tar.bz2
rm eigen.tar.bz2
cd eigen-eigen-5a0156e40feb
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/local
make install
cd ../../
rm -r eigen-eigen-5a0156e40feb
