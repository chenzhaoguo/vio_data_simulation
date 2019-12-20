#!/bin/bash

echo "Configuring and building ..."
if [ ! -d "build" ]; then
  mkdir build
fi

cd ./bin
rm -rf keyframe
rm -rf *.txt

cd ../build
rm -rf *
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4