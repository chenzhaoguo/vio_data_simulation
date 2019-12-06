#!/bin/bash

echo "Configuring and building ..."
if [ ! -d "build" ]; then
  mkdir build
fi

cd build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4