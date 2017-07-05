#!/usr/bin/env bash

# Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
#
# Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
# Institute:   Czech Technical University in Prague
# Created on:  Nov 19, 2013

PKG=`rospack find clopema_libs`

run_doctest() {
    OK=`python $PKG/src_python/clopema_libs/$1 | tail -1`
    echo -e "$1 \t... $OK"
}

for f in $PKG/src_python/clopema_libs/*.py; do
    run_doctest `basename $f`
done
