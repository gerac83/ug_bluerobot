#!/bin/bash

hg clone https://bitbucket.org/Phantoml994/ompl_clopema ~/ompl_clopema
cd ~/ompl_clopema
mkdir -p build/Release
cd build/Release
cmake ../..
make -j "$(nproc)"
