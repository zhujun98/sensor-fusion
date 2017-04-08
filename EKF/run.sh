#!/bin/bash
#\

index=$1

if [ ! $index ]; then index=1; fi

cmake-build-debug/ExtendedKF data/sample-laser-radar-measurement-data-${index}.txt output/output-data-${index}.txt
