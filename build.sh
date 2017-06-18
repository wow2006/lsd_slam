#!/bin/bash

# Thanks to https://gist.github.com/JamieMason/4761049
function program_is_installed {
  # set to 1 initially
  local return_=1
  # set to 0 if not found
  type $1 >/dev/null 2>&1 || { local return_=0; }
  # return value
  echo "$return_"
}

numberOfThreads=$(grep -c ^processor /proc/cpuinfo)

if [ $(program_is_installed ninja) == 1 ]; then
    buildSystem="Ninja"
    buildCommand=ninja
else
    buildSystem="Unix Makefiles"
    buildCommand=make
fi

if [ -d build ]; then
    cd build

    if [ ! -d Debug ]; then
        mkdir Debug
    fi

    if [ ! -d Release ]; then
        mkdir Release
    fi

    cd ..
else
    mkdir build/Release build/Debug -p
fi

# Build Debug
cd build/Debug
cmake -G $buildSystem -DCMAKE_BUILD_TYPE=Debug ../..
$buildCommand -j $numberOfThreads

# Build Release
cd ../Release
cmake -G $buildSystem -DCMAKE_BUILD_TYPE=Release ../..
$buildCommand -j $numberOfThreads
