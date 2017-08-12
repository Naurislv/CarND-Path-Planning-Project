#!/bin/bash

cd build
cmake ..
make -j8
./path_planning