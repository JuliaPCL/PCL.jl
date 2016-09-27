#!/bin/bash -e

for f in $(ls -l ~/.julia/v0.6  | grep PCL | awk '{print $9}' | sort | grep -v "^PCL$" | grep -v "LibPCL" | grep -v "PCLCommon" | awk '{print tolower($1)}')
do
    cp src/common.md src/${f/pcl/}.md
done
