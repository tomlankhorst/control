#!/usr/bin/env sh
# Installs doxygen to $HOME/local

set -ex

FILE=$HOME/local/doxygen    
if [ -f $FILE ]; then
  echo "File $FILE exists. Skipping."
  exit 0
else
  echo "File $FILE does not exist. Building."
fi

wget -O doxygen.tar.gz https://github.com/doxygen/doxygen/archive/Release_1_8_14.tar.gz
tar -xvf doxygen.tar.gz
rm doxygen.tar.gz
cd doxygen-Release_1_8_14
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
mkdir -p $HOME/local
mv bin/* $HOME/local
cd ..
