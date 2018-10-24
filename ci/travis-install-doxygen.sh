#!/usr/bin/env sh
# Installs doxygen to $HOME/local

set -ex

git clone https://github.com/doxygen/doxygen.git
cd doxygen
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
sudo make install
cd ..
rm -rf doxygen
