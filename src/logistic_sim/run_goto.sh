#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SCRIPT_DIR}/../rbkairos_sim/rbkairos_gazebo/models
USE_KAIROS_SIM=false
KAIROS_NAME=rbkairos
#KAIROS_NAME=robot
KAIROS_FRAME=${KAIROS_NAME}_map
SESSION=goto_sim
X_INIT_POS=-1.0
Y_INIT_POS=0.25
MAP=icelab_room
DEBUG=false

function prepare_tmux {
	tmux -2 new-session -d -s $SESSION
	#tmux bind-key X run-shell "./stop_experiment.sh" \\\; kill-session
	tmux bind-key X kill-session
	tmux splitw -h
}

function launch_kairos_sim {
	tmux selectw -t $SESSION:0
	tmux selectp -t $SESSION:0.0
	tmux send-keys -l "roslaunch rbkairos_sim_bringup rbkairos_complete.launch launch_rviz:=true default_map:='icelab_room/icelab_room.yaml' gazebo_world:='worlds/icelab_room.world' x_init_pose_robot_a:=$X_INIT_POS y_init_pose_robot_a:=$Y_INIT_POS"
	tmux send-keys C-m
	echo "Launching Gazebo w/ Kairos..."
	#sleep 10
}

function launch_kairos_agent {
	tmux selectw -t $SESSION:0
	tmux selectp -t $SESSION:0.1
	tmux send-keys "roslaunch logistic_sim agent.launch agents_type:=GoToAgent robot_name:=$KAIROS_NAME agent_name:=patrol_$KAIROS_NAME map_frame:=$KAIROS_FRAME --wait" C-m
}

prepare_tmux

launch_kairos_sim
launch_kairos_agent
date
tmux -2 attach-session -t $SESSION
echo ""
sleep 1