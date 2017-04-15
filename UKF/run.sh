#!/bin/bash
#\

index=$1

if [ ! $index ]; then index=1; fi

if [ -f cmake-build-debug/UnscentedKF ] ; then
    cmake-build-debug/UnscentedKF data/sample-laser-radar-measurement-data-${index}.txt output/basic-output-${index}.txt
else
    echo -e "\nSwitched to ./build directory"
    build/Unscented data/sample-laser-radar-measurement-data-${index}.txt output/basic-output-${index}.txt
fi
