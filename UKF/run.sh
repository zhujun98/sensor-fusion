#!/bin/bash
#\

index=$1

if [ ! $index ]; then
    data=data/obj_pose-laser-radar-synthetic-input.txt
    output=output/basic-output.txt
else
    data=data/sample-laser-radar-measurement-data-${index}.txt
    output=output/basic-output-${index}.txt
fi

if [ -f cmake-build-debug/UnscentedKF ] ; then
    cmake-build-debug/UnscentedKF $data $output
else
    echo -e "\nSwitched to ./build directory"
    build/UnscentedKF $data $output
fi
