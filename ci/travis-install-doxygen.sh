#!/usr/bin/env sh
# Installs doxygen to $HOME/local

set -ex

wget http://ftp.stack.nl/pub/users/dimitri/doxygen-1.8.14.linux.bin.tar.gz
tar -xvf doxygen-1.8.14.linux.bin.tar.gz
rm doxygen-1.8.14.linux.bin.tar.gz
cd doxygen-1.8.14
mkdir -p $HOME/local
cp bin/* $HOME/local
cd ..
rm -r doxygen-1.8.14
