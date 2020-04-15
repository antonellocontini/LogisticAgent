
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Luca Iocchi (2014-2016)
*********************************************************************/

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h> //to get pkg path
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <color_cout.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <queue>

#include "algorithms.hpp" // <<< ALGO
#include "get_graph.hpp"
#include "message_types.hpp"

#include "logistic_sim/Mission.h"
#include "logistic_sim/Token.h"
#include "logistic_sim/RobotReady.h"

using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;
namespace logistic_sim
{

inline bool operator==(const logistic_sim::Mission &A, const logistic_sim::Mission &B)
{
    return A.ID == B.ID ? true : false;
}

inline bool operator<(const logistic_sim::Mission &A, const logistic_sim::Mission &B)
{
    return A.V < B.V ? 1 : 0;
}

} // namespace logistic_sim

namespace agent
{

using uint = unsigned int;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using namespace std;

const std::string PS_path = ros::package::getPath("logistic_sim"); // D.Portugal => get pkg path

class Agent
{
protected:
  int TEAM_SIZE;
  int ID_ROBOT;
  int CAPACITY; //////////////////////////////////////////////////////////////////////////////////////////////

  double xPos[NUM_MAX_ROBOTS]; // tabelas de posições (atençao ao index pro caso
                               // de 1 so robot)
  double yPos[NUM_MAX_ROBOTS]; // tabelas de posições (atençao ao index pro caso
                               // de 1 so robot)
  double thetaPos[NUM_MAX_ROBOTS];
  double lastXpose, lastYpose;

  std::string graph_file, mapname, robotname, mapframe;
  std::string initial_positions;

  uint dimension;      // Graph Dimension
  uint current_vertex; // current vertex
  uint backUpCounter;
  int next_vertex;
  uint initial_vertex; // initial vertex

  int aborted_count, resend_goal_count;

  bool ResendGoal; // Send the same goal again (if goal failed...)
  bool interference;
  bool goal_complete, goal_success;
  bool initialize;
  bool end_simulation;
  bool goal_canceled_by_user;

  double last_interference;
  double *last_visit;

  vertex *vertex_web;
  std::vector<uint> path;
 
  tf::TransformListener *listener;
  MoveBaseClient *ac; // action client for reaching target goals

  ros::Subscriber odom_sub;

  // used for recovery behaviors to send commands to base
  ros::Publisher cmd_vel_pub;

  ros::Publisher token_pub;
    ros::Subscriber token_sub;

    bool reached_home = false;

    // TODO: read from parameter file (see task planners)
    std::map<std::string, uint> map_src = {{"model6", 13}, {"grid", 7}, {"icelab", 22}, {"icelab_black", 2}, {"model5", 6}};
    std::map<std::string, std::vector<uint>> map_dsts = {{"model6", {18, 23, 28}}, {"grid", {16, 17, 18}},
                                                         {"icelab", {10, 13, 16}}, {"icelab_black", {26, 33, 42}},
                                                         {"model5", {11, 16, 21}}};
    
    // here are kept pickup and delivery vertices
    uint src_vertex;
    std::vector<uint> dsts_vertex;

public:
  Agent()
  {
    listener = NULL;
    next_vertex = -1;
    initialize = true;
    end_simulation = false;
    ac = NULL;
  }

  virtual void init(int argc, char **argv);
  void set_map_endpoints(ros::NodeHandle &nh);
  void ready();
  virtual void run();
  void readParams(); // read ROS parameters
 
  void getRobotPose(int robotid, float &x, float &y, float &theta);
  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
  
  void sendGoal(int next_vertex);
  void cancelGoal();
  
  virtual void goalDoneCallback(const actionlib::SimpleClientGoalState &state,
                        const move_base_msgs::MoveBaseResultConstPtr &result);
  virtual void goalActiveCallback();
  virtual void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

  virtual void token_callback(const logistic_sim::TokenConstPtr &msg) = 0;

  void backup();
};

} // namespace agent
