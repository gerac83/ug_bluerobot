#!/bin/bash

NEW_OMPL_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" ../ )" && pwd)/ompl"
echo $NEW_OMPL_DIR
hg clone https://bitbucket.org/Phantoml994/ompl_clopema $NEW_OMPL_DIR
cd $NEW_OMPL_DIR
hg update indigo-devel
mkdir -p build/Release
cd build/Release
cmake ../..
make -j "$(nproc)"
