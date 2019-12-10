#!/bin/bash

SESSION=log_sim
MAP=grid
NROBOTS=4
INITPOS=default
ALG=OnlineAgent
LOC=AMCL
NAV=ros
GWAIT=0
COMMDELAY=0.2
TERM=xterm 
TIMEOUT=1800
CUSTOM_STAGE=false
SPEEDUP=3.0
CAPACITY=3
TP_NAME=OnlineTaskPlanner
GEN=rand
PERM=true
DEBUG=true
MISSIONS_FILE=10.txt
NRUNS=1

function prepare_tmux {
	n=$(( NROBOTS - 1 ))
	tmux -2 new-session -d -s $SESSION
	tmux bind-key X run-shell "./stop_experiment.sh" \\\; kill-session
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
	tmux send-keys "roscore &" C-m
	echo "Launching roscore..."
	sleep 8
	echo "Setting ROS parameters..."
	tmux send-keys "rosparam set /use_sim_time True" C-m
	tmux send-keys "rosparam set /goal_reached_wait $GWAIT" C-m
	tmux send-keys "rosparam set /communication_delay $COMMDELAY" C-m
	tmux send-keys "rosparam set /navigation_module $NAV" C-m
	tmux send-keys "rosparam set /initial_positions $INITPOS" C-m
	IPOSES=$(cat params/initial_poses.txt | grep "$MAP"_"$NROBOTS" | grep -o "\[.*\]")
	tmux send-keys "./setinitposes.py $MAP 	\"$IPOSES\"" C-m
	sleep 1
}

function launch_stage {
	tmux selectw -t $SESSION:0
	tmux selectp -t $SESSION:0.0
	tmux send-keys "roslaunch logistic_sim map.launch map:=$MAP --wait" C-m
	echo "Launching Stage..."
	sleep 3
}

function launch_robots {
	tmux selectw -t $SESSION:1
	n=$(( NROBOTS - 1 ))
	for i in $(seq 0 $n); do
		tmux selectp -t $SESSION:1.$i
		tmux send-keys "roslaunch logistic_sim robot.launch robotname:=robot_$i mapname:=$MAP use_amcl:=true use_move_base:=true --wait" C-m
		echo "Robot $i launched"
		sleep 1
	done
	tmux select-layout tiled

	echo "Waiting for robots to get ready..."
	sleep 5
}

function launch_taskplanner {
	tmux selectw -t $SESSION:3
	if [ $DEBUG = true ] ; then
		if [ -f "commands_taskplanner.txt" ] ; then
			tmux send-keys "rosrun --prefix 'gdb -x commands_taskplanner.txt --args' logistic_sim $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY $MISSIONS_FILE" C-m
		else
			tmux send-keys "rosrun --prefix 'gdb -ex run --args' logistic_sim $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY $MISSIONS_FILE" C-m
		fi
	else
		tmux send-keys "rosrun logistic_sim $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY $MISSIONS_FILE" C-m
	fi

	echo "Launching TaskPlanner $TP_NAME..."
	sleep 5
}

function launch_agents {
	tmux selectw -t $SESSION:2
	echo "Launching agents..."
	if [ $DEBUG = true ] ; then
		echo "Debug mode activated, running into gdb..."
	fi
	for i in $(seq 0 $n); do
		tmux selectp -t $SESSION:2.$i
		if [ $DEBUG = true ] ; then
			if [ -f "commands_all.txt" ] ; then
				tmux send-keys "rosrun --prefix 'gdb -q -x commands_all.txt --args ' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
			elif [ -f "commands_$i.txt" ] ; then
				tmux send-keys "rosrun --prefix 'gdb -q -x commands_$i.txt --args ' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
			else
				tmux send-keys "rosrun --prefix 'gdb -q -ex run --args' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
			fi
		else
			tmux send-keys "rosrun logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
		fi
		echo "$ALG $i launched"
		sleep 1
	done
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
launch_stage
launch_robots
launch_taskplanner
launch_agents
set_footprints
date
tmux -2 attach-session -t $SESSION
echo ""
sleep 1
