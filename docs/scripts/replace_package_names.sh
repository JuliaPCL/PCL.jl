#!/bin/bash

set -e

for f in $(ls -l ~/.julia/v0.6  | grep PCL | awk '{print $9}' | sort | grep -v "^PCL$" | grep -v "PCLCommon" | grep -v "LibPCL")
do
    lowername=$(echo $f | tr '[:upper:]' '[:lower:]')
    dstname=src/${lowername/pcl/}.md
    echo $dstname
    sed -i '' -e "s/PCLCommon/$f/g" $dstname
done
