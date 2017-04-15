#!/bin/bash
#\

index=$1

if [ ! $index ]; then index=1; fi

if [ -f cmake-build-debug/ExtendedKF ] ; then
    cmake-build-debug/ExtendedKF data/sample-laser-radar-measurement-data-${index}.txt output/basic-output-${index}.txt
else
    echo -e "\nSwitched to ./build directory"
    build/ExtendedKF data/sample-laser-radar-measurement-data-${index}.txt output/basic-output-${index}.txt
fi
