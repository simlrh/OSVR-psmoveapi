#!/bin/sh

git submodule update --init --recursive
cd vendor/psmoveapi
git update-index --assume-unchanged src/tracker/platform/camera_control_linux.c
git apply ../../psmoveapi-patch
cd ../..
mkdir build
cd build
cmake ..
make
