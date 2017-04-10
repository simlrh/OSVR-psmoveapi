#!/bin/sh

git submodule update --init --recursive
cd vendor/psmoveapi
git apply -vvv ../../psmoveapi.patch
cd ../..
mkdir -p build
cd build
cmake ..
make -j$(nproc)
