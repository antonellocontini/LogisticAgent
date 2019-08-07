
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

#include "algorithms.hpp" // <<< ALGO
#include "getgraph.hpp"
#include "message_types.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define NUM_MAX_ROBOTS 32
#define INTERFERENCE_DISTANCE 2.5
#define SHARE_MSG 33
#define DELTA_TIME_SEQUENTIAL_START 15
#define SIMULATE_FOREVER false // WARNING: Set this to false, if you want a finishing condition.
#define DBG true

#if DBG
#include <task_planner/Task.h>
#include <patrolling_sim/MissionRequest.h>
#include <patrolling_sim/TaskRequest.h>
// #include <patrolling_sim/Vertex.h>
// #include <patrolling_sim/VertexWeb.h>
//#include <tcp_interface/RCOMMessage.h>
#endif

namespace patrolagent
{

struct Task
{
  bool take;
  int item;
  int order;
  int demand;
  int dst;
  int path_distance;
  std::vector<uint> trail;
};

ostream &operator<<(ostream &os, const Task &t)
{
  os << "\nRoute: \n";
  for (auto i = 0; t.trail.size(); i++)
  {
    os << t.trail[i] << " ";
  }
  os << "\n";
}

using uint = unsigned int;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using namespace std;

const std::string PS_path = ros::package::getPath("patrolling_sim"); // D.Portugal => get pkg path

class PatrolAgent
{
protected:
  int TEAMSIZE;
  int ID_ROBOT;
  int CAPACITY; //////////////////////////////////////////////////////////////////////////////////////////////

  double xPos[NUM_MAX_ROBOTS]; // tabelas de posições (atençao ao index pro caso
                               // de 1 so robot)
  double yPos[NUM_MAX_ROBOTS]; // tabelas de posições (atençao ao index pro caso
                               // de 1 so robot)
  double thetaPos[NUM_MAX_ROBOTS];
  double lastXpose, lastYpose;

  std::string graph_file, mapname, robotname;
  std::string initial_positions;

  uint dimension;      // Graph Dimension
  uint current_vertex; // current vertex
  uint backUpCounter;
  int next_vertex;
  uint initial_vertex; // initial vertex
  int current_dim_path;

  int aborted_count, resend_goal_count;
  int interference_cnt;

  bool ResendGoal; // Send the same goal again (if goal failed...)
  bool interference;
  bool goal_complete;
  bool initialize;
  bool end_simulation;
  bool goal_canceled_by_user;

  bool OK = false;
  bool first = true;

  double last_interference;
  double *instantaneous_idleness; // local idleness
  double *last_visit;
  double goal_reached_wait, communication_delay, last_communication_delay_time, lost_message_rate;

  vertex *vertex_web;

  std::vector<int> vresults; // results exchanged among robots
  std::vector<int> shared_array;

  tf::TransformListener *listener;
  MoveBaseClient *ac; // action client for reaching target goals

  ros::Subscriber odom_sub, positions_sub;
// /------------------------------------------------------------------------
#if DBG
  ros::Subscriber sub_to_task_planner_mission;
  ros::Publisher pub_to_task_planner_needtask;
  ros::Publisher pub_to_task_planner_needmission;
  ros::Publisher pub_to_task_planner_init;

  ros::Subscriber sub_to_task_planner_init;

  // ros::Subscriber rcom_sub;
  // ros::Publisher rcom_pub;
#endif
  ros::Subscriber pose_sub;

  ros::Publisher positions_pub;
  ros::Subscriber results_sub;
  ros::Publisher results_pub;
  ros::Publisher cmd_vel_pub;

  ros::Publisher pub_broadcast_msg;
  ros::Subscriber sub_broadcast_msg;

  std::vector<Task> mission;

  bool *ok;
  bool at_home = false;
  uint id_vertex = 0;
  uint id_task = 0;
  uint route_dimension;
  uint loading[6] = {2, 1, 0, 3, 5, 6};
  uint downloading[5] = {4, 7, 10, 13, 16};
  uint route_to_src[5] = {1, 0, 3, 5, 6};

public:
  PatrolAgent()
  {
    listener = NULL;
    next_vertex = -1;
    initialize = true;
    end_simulation = false;
    ac = NULL;
  }

  virtual void init(int argc, char **argv);
  void readParams(); // read ROS parameters
  // void initialize_node();
  void update_idleness(); // local idleness

  void send_initialize_msg();
  void init_Callback(const std_msgs::Int16MultiArrayConstPtr &msg);
  // void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  virtual void run();
  void ready();
  virtual void onGoalComplete(); // what to do when a goal has been reached

  void getRobotPose(int robotid, float &x, float &y, float &theta);
  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

  void sendGoal(int next_vertex);
  void sendMissionGoal(vector<uint> mission);
  void cancelGoal();

  void goalDoneCallback(const actionlib::SimpleClientGoalState &state,
                        const move_base_msgs::MoveBaseResultConstPtr &result);
  void goalActiveCallback();
  void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

  void send_goal_reached();
  void send_task_reached();
  bool check_interference(int ID_ROBOT);
  void do_interference_behavior();
  // void backup();

  // void onGoalNotComplete(); // what to do when a goal has NOT been reached
  // (aborted)

  // Events
  virtual void processEvents(); // processes algorithm-specific events

  // Robot-Robot Communication
  void send_positions();
  void receive_positions();
  virtual void send_results();    // when goal is completed
  virtual void receive_results(); // asynchronous call
  void do_send_message(std_msgs::Int16MultiArray &msg);
  void send_interference();
  void send_resendgoal();
  void positionsCB(const nav_msgs::Odometry::ConstPtr &msg);
  void resultsCB(const std_msgs::Int16MultiArray::ConstPtr &msg);
#if DBG
  // void resultsCB(const tcp_interface::RCOMMessage::ConstPtr &msg);
#endif
  // Must be implemented by sub-classes
  virtual int compute_next_vertex();

  //--------------------------------------------------------------------------

  int ccor(vector<uint> v);

  void calc_route_to_src();
  void init_agent();
#if DBG
  void request_Task();
  void request_Mission();
  void receive_mission_Callback(const task_planner::TaskConstPtr &msg);
  //   ^ nuovi messaggi di task NB route[];
#endif
  void broadcast_msg_Callback(const std_msgs::Int16MultiArray::ConstPtr &msg);
  //----------------------------------------------------------------------
};

} // namespace patrolagent

#include "impl/PatrolAgent.i.hpp"
