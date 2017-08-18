#!/bin/bash

# create directory if it does not exist
mkdir -p build
cd build
cmake ..
# build code using 8 CPU cores
make -j8
./path_planning