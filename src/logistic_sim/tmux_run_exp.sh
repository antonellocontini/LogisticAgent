#!/bin/bash

SESSION=log_sim
MAP=grid
NROBOTS=4
INITPOS=default
ALG=CFreeAgent
LOC=AMCL
NAV=ros
GWAIT=0
COMMDELAY=0.2
TERM=xterm 
TIMEOUT=1800
CUSTOM_STAGE=false
SPEEDUP=3.0
CAPACITY=3
TP_NAME=TaskPlanner
GEN=uniform
DEBUG=false

clear
tmux -2 new-session -d -s $SESSION
tmux bind-key X run-shell "./stop_experiment.sh" \\\; kill-session

tmux send-keys "roscore &" C-m
echo "Launching roscore..."
sleep 3
echo "Setting ROS parameters..."
tmux send-keys "rosparam set /use_sim_time True" C-m
tmux send-keys "rosparam set /goal_reached_wait $GWAIT" C-m
tmux send-keys "rosparam set /communication_delay $COMMDELAY" C-m
tmux send-keys "rosparam set /navigation_module $NAV" C-m
tmux send-keys "rosparam set /initial_positions $INITPOS" C-m
IPOSES=$(cat params/initial_poses.txt | grep "$MAP"_"$NROBOTS" | grep -o "\[.*\]")
tmux send-keys "./setinitposes.py $MAP 	\"$IPOSES\"" C-m
tmux send-keys "roslaunch logistic_sim map.launch map:=$MAP" C-m
echo "Launching Stage..."
sleep 3

tmux new-window -t $SESSION:1 -n 'Robots'
echo "Launching robots..."

n=$(( NROBOTS - 1 ))
for i in $(seq 0 $n); do
	tmux splitw
	tmux select-pane -t $SESSION:1.$i
	tmux send-keys "roslaunch logistic_sim robot.launch robotname:=robot_$i mapname:=$MAP use_amcl:=true use_move_base:=true" C-m
	tmux select-layout tiled
	echo "Robot $i launched"
	sleep 1
done
tmux kill-pane -t $SESSION:1.$NROBOTS
tmux select-layout tiled

echo "Waiting for robots to get ready..."
sleep 5

tmux new-window -t $SESSION:3 -n 'TaskPlanner'

if [ $DEBUG = true ] ; then
	tmux send-keys "rosrun --prefix 'gdb -ex run --args' task_planner $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY" C-m
else
	tmux send-keys "rosrun task_planner $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY" C-m
fi
#tmux send-keys "rosrun task_planner $TP_NAME $MAP $ALG $NROBOTS $GEN $CAPACITY" C-m
echo "Launching TaskPlanner $TP_NAME..."
sleep 5

tmux new-window -t $SESSION:2 -n 'Agents'

echo "Launching agents..."
if [ $DEBUG = true ] ; then
	echo "Debug mode activated, running into gdb..."
fi
for i in $(seq 0 $n); do
	tmux splitw
	tmux select-pane -t $SESSION:2.$i
	if [ $DEBUG = true ] ; then
		if [ -f "commands_$i.txt" ] ; then
			tmux send-keys "rosrun --prefix 'gdb -x commands_$i.txt --args ' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
		else
			tmux send-keys "rosrun --prefix 'gdb -ex run --args' logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
		fi
	else
		tmux send-keys "rosrun logistic_sim $ALG __name:=patrol_robot$i $MAP $i robot_$i $CAPACITY $NROBOTS" C-m
	fi
	tmux select-layout tiled
	echo "$ALG $i launched"
	sleep 1
done
tmux kill-pane -t $SESSION:2.$NROBOTS
tmux select-layout tiled

tmux select-window -t $SESSION:0

echo "Setting stage footprints..."
tmux splitw -h
tmux send-keys "rostopic pub /stageGUIRequest std_msgs/String \"data: 'footprints'\" --once" C-m
tmux send-keys "rostopic pub /stageGUIRequest std_msgs/String \"data: 'speedup_$SPEEDUP'\" --once" C-m

tmux select-window -t $SESSION:2

tmux -2 attach-session -t $SESSION
