#!/bin/bash

CUR_DIR=$PWD
DEVEL_DIR=$PWD/devel

cd src/edlut/EDLUT_source_code

mkdir -p $DEVEL_DIR
mkdir -p $DEVEL_DIR/bin              # Fix for make install error

autoconf
./configure --prefix=$DEVEL_DIR
make install

cd $CUR_DIR

catkin_make "$@"
