#!/usr/bin/env sh
# Installs doxygen to $HOME/local

set -ex

wget -O doxygen.tar.gz https://github.com/doxygen/doxygen/archive/Release_1_8_14.tar.gz
tar -xvf doxygen.tar.gz
rm doxygen.tar.gz
cd doxygen-Release_1_8_14
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
make install
cd ..
rm -rf doxygen-Release_1_8_14
