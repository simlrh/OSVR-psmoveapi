#!/bin/sh

git submodule update --init --recursive
cd vendor/psmoveapi
git update-index --assume-unchanged src/tracker/platform/camera_control_linux.c
git apply -vvv ../../psmoveapi-patch
git apply -vvv ../../add-libv4l2-module.patch
git apply -vvv ../../fix-opencv-headers.patch
cd ../..
mkdir -p build
cd build
cmake ..
make -j$(nproc)
