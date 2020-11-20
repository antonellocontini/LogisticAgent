#!/bin/bash

#ROS_MASTER_URI=http://SXLSK-190911AA:11311
KAIROS_NAME=fufi
KAIROS_FRAME=fufi_map
KAIROS_ORDER=0
INTERACTIVE_MODE=true
MAP=ice_full_20201005
NROBOTS=2
ALG=OnlineDCOPAgent
CAPACITY=3
DEBUG=false

# function requires two parameters
# 1. kairos name
# 2. kairos map frame
# 3. order value
function launch_real_kairos_agent {
	if [ ! $# -eq 3 ]; then
		return 1
	fi
	if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
		return 2
	fi
	echo "Launching agent..."
	roslaunch logistic_sim agent.launch mapname:=$MAP map_frame:=$2 agents_type:=$ALG agents_number:=$NROBOTS robots_capacity:=$CAPACITY debug_mode:=$DEBUG robot_order:=$3 robot_name:=$1 agent_name:=patrol_robot$3 interactive_mode:=$INTERACTIVE_MODE --wait
}

launch_real_kairos_agent "$KAIROS_NAME" "$KAIROS_FRAME" "$KAIROS_ORDER"