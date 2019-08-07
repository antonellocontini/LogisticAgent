#!/bin/bash
./../py/setinitposes.py model3 "[9.00, 5.30]" &
rosrun patrolling_sim monitor model3 LOGA 1 &
roslaunch patrolling_sim map.launch map:=model3 &
roslaunch patrolling_sim robot.launch robotname:=robot_0 mapname:=model3 use_amcl:=true use_move_base:=true &