#!/bin/bash

for f in $(ls -l ~/.julia/v0.6  | grep PCL | awk '{print $9}' | sort | grep -v "^PCL$" | grep -v "LibPCL" | awk '{print $1}')
do
    ff=$(echo $f | tr '[:upper:]' '[:lower:]')
    echo "\"${f/PCL/}\" => \"api/${ff/pcl/}.md\""
done
