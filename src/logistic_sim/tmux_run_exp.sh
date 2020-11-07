#!/bin/bash

#ROS_MASTER_URI=http://SXLSK-190911AA:11311
USE_KAIROS_SIM=false
USE_KAIROS=false
#KAIROS_NAME=rbkairos
KAIROS_NAME=robot
KAIROS_FRAME=robot_map
INTERACTIVE_MODE=false
SESSION=log_sim
MAP=icelab_black
NROBOTS=2
INITPOS=default
ALG=OnlineDCOPAgent
#LOC=AMCL
LOC=fake_localization
NAV=ros
SPEEDUP=3.0
CAPACITY=3
TP_NAME=OnlineDCOPTaskPlanner
#GEN=null
GEN=file
DEBUG=false
MISSIONS_FILE=1.txt
NRUNS=1

if [ "$USE_KAIROS_SIM" = "true" ]; then
	NROBOTS=1	# only one kairos supported
fi

function prepare_tmux {
	n=$(( NROBOTS - 1 ))
	tmux -2 new-session -d -s $SESSION
	#tmux bind-key X run-shell "./stop_experiment.sh" \\\; kill-session
	tmux bind-key X kill-session
	tmux splitw -h
	tmux new-window -t $SESSION:1 -n 'Robots'
	for i in $(seq 0 $n); do
		tmux splitw
		tmux select-layout tiled
	done
	tmux kill-pane -t $SESSION:1.$NROBOTS
	tmux new-window -t $SESSION:2 -n 'Agents'
	for i in $(seq 0 $n); do
		tmux splitw
		tmux select-layout tiled
	done
	tmux kill-pane -t $SESSION:2.$NROBOTS
	tmux new-window -t $SESSION:3 -n 'TaskPlanner'
}

function launch_ros {
	tmux selectw -t $SESSION:0
	tmux selectp -t $SESSION:0.0
	if [ "$USE_KAIROS" = "false" ]; then
		tmux send-keys "roscore &" C-m
		echo "Launching roscore..."
	fi
		until rostopic list &> /dev/null; do sleep 1; done
		echo "Setting ROS parameters..."
	if [ "$USE_KAIROS" = "false" ]; then
		tmux send-keys "rosparam set /use_sim_time True" C-m
	fi
		tmux send-keys "rosparam set /navigation_module $NAV" C-m
	tmux send-keys "rosparam set /initial_positions $INITPOS" C-m
	IPOSES=$(cat params/initial_poses.txt | grep "$MAP"_"$NROBOTS" | grep -o "\[.*\]")
	tmux send-keys "./setinitposes.py $MAP 	\"$IPOSES\"" C-m
	sleep 1
}

function launch_kairos_sim {
	tmux selectw -t $SESSION:0
	tmux selectp -t $SESSION:0.0
	tmux send-keys "roslaunch rbkairos_sim_bringup rbkairos_complete.launch launch_rviz:=true default_map:='icelab_room/icelab_room.yaml' gazebo_world:='/home/antonello/rbkairos_workspace/src/rbkairos_sim/rbkairos_gazebo/worlds/icelab_room.world' x_init_pose_robot_a:=-1.0 y_init_pose_robot_a:=0.25 --wait" C-m
	echo "Launching Gazebo w/ Kairos..."
	sleep 10
}

function launch_kairos_planner_agents {
	tmux selectw -t $SESSION:1
	tmux selectp -t $SESSION:1.0
	tmux send-keys "roslaunch logistic_sim kairos.launch planner_type:=$TP_NAME agents_type:=$ALG robot_name:=$KAIROS_NAME agent_name:=patrol_$KAIROS_NAME mapname:=$MAP gen_type:=$GEN robot_order:=0 interactive_mode:=$INTERACTIVE_MODE --wait" C-m
}

function launch_stage {
	if [ "$USE_KAIROS" = "false" ]; then 
		tmux selectw -t $SESSION:0
		tmux selectp -t $SESSION:0.0
		tmux send-keys "roslaunch logistic_sim map.launch map:=$MAP --wait" C-m
		echo "Launching Stage..."
		sleep 3
	fi
}

function launch_robots {
	tmux selectw -t $SESSION:1
	n=$(( NROBOTS - 1 ))
	for i in $(seq 0 $n); do
		tmux selectp -t $SESSION:1.$i
		if [ "$USE_KAIROS" = "false" ]; then
			if [ "$LOC" = "AMCL" ]; then
				tmux send-keys "roslaunch logistic_sim robot.launch robotname:=robot_$i mapname:=$MAP use_amcl:=true use_move_base:=true --wait" C-m
			else
				tmux send-keys "roslaunch logistic_sim robot_fake_loc.launch robotname:=robot_$i mapname:=$MAP use_amcl:=true use_move_base:=true --wait" C-m
			fi
			echo "Robot $i launched"
			sleep 1
		fi
	done
	tmux select-layout tiled

	echo "Waiting for robots to get ready..."
	sleep 5
}

function launch_taskplanner {
	tmux selectw -t $SESSION:3
	tmux send-keys "roslaunch logistic_sim task_planner.launch planner_type:=$TP_NAME mapname:=$MAP agents_type:=$ALG agents_number:=$NROBOTS gen_type:=$GEN robots_capacity:=$CAPACITY missions_file:=$MISSIONS_FILE debug_mode:=true --wait" C-m
	# if [ $DEBUG = true ] ; then
	# 	if [ -f "commands_taskplanner.txt" ] ; then
	# 		echo "Debug mode activated, gdb commands from file..."
	# 		tmux send-keys "rosrun --prefix 'gdb -x commands_taskplanner.txt --args' logistic_sim $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY $MISSIONS_FILE; tmux kill-session" C-m
	# 	else
	# 		tmux send-keys "rosrun --prefix 'gdb -ex run --args' logistic_sim $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY $MISSIONS_FILE; tmux kill-session" C-m
	# 	fi
	# else
	# 	tmux send-keys "rosrun logistic_sim $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY $MISSIONS_FILE; tmux kill-session" C-m
	# fi

	echo "Launching TaskPlanner $TP_NAME..."
	sleep 5
}

function launch_real_kairos_agent {
	tmux selectw -t $SESSION:2
	echo "Launching agent..."
	tmux selectp -t $SESSION:2.$i
	tmux send-keys "roslaunch logistic_sim agent.launch mapname:=$MAP map_frame:=$KAIROS_FRAME agents_type:=$ALG agents_number:=$NROBOTS robots_capacity:=$CAPACITY debug_mode:=$DEBUG robot_order:=$i robot_name:=$KAIROS_NAME agent_name:=patrol_robot$i interactive_mode:=$INTERACTIVE_MODE --wait" C-m
	tmux select-layout tiled
}

function launch_agents {
	tmux selectw -t $SESSION:2
	echo "Launching agents..."
	for i in $(seq 0 $n); do
		tmux selectp -t $SESSION:2.$i
		tmux send-keys "roslaunch logistic_sim agent.launch mapname:=$MAP agents_type:=$ALG agents_number:=$NROBOTS robots_capacity:=$CAPACITY debug_mode:=$DEBUG robot_order:=$i robot_name:=robot_$i agent_name:=patrol_robot$i interactive_mode:=$INTERACTIVE_MODE --wait" C-m
	done
	# if [ $DEBUG = true ] ; then
	# 	echo "Debug mode activated, running into gdb..."
	# fi
	# for i in $(seq 0 $n); do
	# 	tmux selectp -t $SESSION:2.$i
	# 	if [ $DEBUG = true ] ; then
	# 		if [ -f "commands_all.txt" ] ; then
	# 			tmux send-keys "rosrun --prefix 'gdb -q -x commands_all.txt --args ' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
	# 		elif [ -f "commands_$i.txt" ] ; then
	# 			tmux send-keys "rosrun --prefix 'gdb -q -x commands_$i.txt --args ' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
	# 		else
	# 			tmux send-keys "rosrun --prefix 'gdb -q -ex run --args' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
	# 		fi
	# 	else
	# 		tmux send-keys "rosrun logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
	# 	fi
	# 	echo "$ALG $i launched"
	# 	sleep 1
	# done
	tmux select-layout tiled
}

function set_footprints {
	tmux selectw -t $SESSION:0
	tmux selectw -t $SESSION:0.1
	echo "Setting stage footprints..."
	tmux send-keys "rostopic pub /stageGUIRequest std_msgs/String \"data: 'footprints'\" --once" C-m
	tmux send-keys "rostopic pub /stageGUIRequest std_msgs/String \"data: 'speedup_$SPEEDUP'\" --once" C-m
	tmux selectw -t $SESSION:2
}

prepare_tmux
launch_ros

if [ "$USE_KAIROS_SIM" = "true" ]; then
	launch_kairos_sim
	launch_kairos_planner_agents
else
	launch_stage
	launch_robots
	launch_taskplanner
	if [ "$USE_KAIROS" = "true" ]; then
		launch_real_kairos_agent
	else
		launch_agents
	fi
	set_footprints
fi
date
tmux -2 attach-session -t $SESSION
echo ""
sleep 1

# for i in $(seq 2 2)
# do
# 	MISSIONS_FILE="$i.txt"
# 	echo "MISSIONS_FILE:" "$MISSIONS_FILE"
# 	prepare_tmux
# 	launch_ros
# 	launch_stage
# 	launch_robots
# 	launch_taskplanner
# 	launch_agents
# 	set_footprints
# 	date
# 	tmux -2 attach-session -t $SESSION
# 	echo ""
# 	sleep 1
# done

# ALG=OnlineAgent
# TP_NAME=OnlineTaskPlanner
# for i in $(echo 2)
# do
# 	NROBOTS=$i
# 	prepare_tmux
# 	launch_ros
# 	launch_stage
# 	launch_robots
# 	launch_taskplanner
# 	launch_agents
# 	set_footprints
# 	date
# 	tmux -2 attach-session -t $SESSION
# 	echo ""
# 	sleep 1
# done

# ALG=OnlineCentralizedAgent
# TP_NAME=OnlineGlobalTaskPlanner
# for i in $(echo 2)
# do
# 	NROBOTS=$i
# 	prepare_tmux
# 	launch_ros
# 	launch_stage
# 	launch_robots
# 	launch_taskplanner
# 	launch_agents
# 	set_footprints
# 	date
# 	tmux -2 attach-session -t $SESSION
# 	echo ""
# 	sleep 1
# done

# ALG=OnlineCentralizedAgent
# TP_NAME=OnlineGreedyTaskPlanner
# for i in $(echo 2 4 6)
# do
# 	NROBOTS=$i
# 	prepare_tmux
# 	launch_ros
# 	launch_stage
# 	launch_robots
# 	launch_taskplanner
# 	launch_agents
# 	set_footprints
# 	date
# 	tmux -2 attach-session -t $SESSION
# 	echo ""
# 	sleep 1
# done

# NROBOTS=4
# for i in $(seq 1 10)
# do
# 	MISSIONS_FILE="$i.txt"
# 	prepare_tmux
# 	launch_ros
# 	launch_stage
# 	launch_robots
# 	launch_taskplanner
# 	launch_agents
# 	set_footprints
# 	date
# 	tmux -2 attach-session -t $SESSION
# 	echo ""
# 	sleep 1
# done

# NROBOTS=6
# for i in $(echo 6 10)
# do
# 	MISSIONS_FILE="$i.txt"
# 	prepare_tmux
# 	launch_ros
# 	launch_stage
# 	launch_robots
# 	launch_taskplanner
# 	launch_agents
# 	set_footprints
# 	date
# 	tmux -2 attach-session -t $SESSION
# 	echo ""
# 	sleep 1
# done
