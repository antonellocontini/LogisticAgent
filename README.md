# logistic_sim
A heaviliy modified version of the [patrolling_sim](http://wiki.ros.org/patrolling_sim) module to solve the multi-item variant of the MAPD (Multi-Agent Pickup and Delivery) problem. This package implements an allocation and coordination scheme based on a token passing algorithm that enables robot to find collision-free paths to solve MAPD instances.

## Getting started
This package has been tested on ROS Kinetic and Ubuntu 16.04

### Prerequisites
This package uses the ROS navigation stack and the Stage simulator (all available in the ROS repos).

To run the CLI you need `tmux`.

### Compiling
To compile just `cd` into the repo and run `catkin_make`, if compilation fails you're probably missing some dependencies.

After compiling the code run `source devel/setup.bash` to add the workspace to the environment (it is good practice to add this script to your `.bashrc` file).

### Running
There are two ways to launch the simulation, one using `tmux` (recommended) and another using `xterm`

#### tmux
To run a simulation head into the `src/logistic_sim` directory and run `./tmux_run_exp.sh`.

This script uses `tmux` to organize the terminals of the different nodes so it is recommended to know at least how to switch between windows and panes. The nodes are organized in different windows:
* roscore and free terminal
* move_base nodes
* agents nodes
* task planner node

The script lands on the agents window.

If you desire to terminate the simulation prematurely press ctrl-b X (assuming you're using the default tmux prefix key).

#### xterm
Inside `src/logistic_sim` directory run `./run_exp.sh` to start the simulation. To stop the simulation run `./stop_experiment.sh`

#### Parameters
Inside the script there are many parameters that can be configured:

The `ALG` parameter sets which agents algorithm to run:
* `OnlineDCOPAgent` implements the online allocation and planning algorithm
* `OnlineCentralizedAgent` is used in conjunction with two different centralized approaches (one with optimal allocation and the other with a greedy strategy)

The `TP_NAME` parameter sets the task planner:
* `OnlineDCOPTaskPlanner` must be used together with `OnlineDCOPAgent`
* `OnlineGlobalTaskPlanner` and `OnlineGreedyTaskPlanner` must be used with `OnlineCentralizedAgent` and they implements the two approaches aforementioned.

The `GEN` parameter sets how the tasks are generated:
* `uniform` creates tasks uniformly distributed in terms of delivery locations and demand values
* `rand` creates random tasks
* `file` reads the tasks from the file specified in the `MISSIONS_FILE` parameter, this file must be contained in the `missions` directory
* `null` does not create tasks, they must be provided calling the `AddMissions` service

An example for the `AddMissions` service:
`rosservice call /add_missions "[{DSTS:[42]},{DSTS:[26,42]}]"`
In this example two missions are inserted, one with a single delivery location and one with two delivery locations

The `NROBOTS` parameter sets the number of robots to run in the simulation, currently only configurations of 2, 4 or 6 robots are defined, but different ones can be made in the `params/initial_poses.txt` file. This file contains the starting positions of the robots for each map.

In this moment only the `icelab_black`,`icelab_room`,`grid` and `model5` maps are working.

## RB-KAIROS
If you want to use the kairos simulator the `USE_KAIROS_SIM` flag must be set to `true`.
To use the kairos simulator its packages must be installed, instructions can be found [here](https://github.com/RobotnikAutomation/rbkairos_sim).

If you want to use the real robot then the `USE_KAIROS` flag must be set to `true`.
In this case the `ROS_MASTER_URI` must be set to the robot's one, because roscore runs on the robot.

## Debugging
If the `DEBUG` flag is set to `True` the agents and the task planner will run inside gdb. Custom gdb commands can be given to the robots and the taskplanner by creating specific named files inside the `src/logistic_sim` directory:
* For the taskplanner create a `commands_taskplanner.txt` file
* For a single agent create a `commands_<agent_id>.txt` file, where the agent id can go from 0 to `NROBOTS - 1`
* To send commands to all agents create `commands_all.txt` file
