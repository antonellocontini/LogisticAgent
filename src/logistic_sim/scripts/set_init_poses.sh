#!/bin/sh

MAP=$1
NROBOTS=$2

pwd
IPOSES=$(cat ../params/init_poses.txt | grep "$MAP"_"$NROBOTS" | grep -o "\[.*\]")
echo "$IPOSES"
../setinitposes.py $MAP "$IPOSES"