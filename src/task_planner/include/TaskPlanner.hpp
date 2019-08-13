#pragma once
#include <color_cout.hpp> //lib
// #include <partition.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <ros/package.h> //to get pkg path
#include <ros/ros.h>
#include <sys/stat.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#include <logistic_sim/TaskRequest.h>
#include <logistic_sim/Task.h>
#include <logistic_sim/Token.h>
#include <task_planner/Task.h>

#include "struct_task.hpp"
#include "struct_vertex.hpp"
#include "message_types.hpp"
#include "get_graph.hpp"
#include "color_cout.hpp"


namespace taskplanner {


const std::string PS_path = ros::package::getPath("logistic_sim");

class TaskPlanner {
public:
  TaskPlanner(ros::NodeHandle &nh_);
  ~TaskPlanner(){};

  vertex *vertex_web;

  uint TEAM_CAPACITY = 0;
  std::vector<uint> CAPACITY;
  uint TEAM_SIZE = 0;
  uint nTask = 0;
  uint id = 0;
  uint src_vertex = 6;
  uint dst_vertex[3] = {11, 16, 21};
  // per ora statici senza funzioni
  uint p_11[8] = {6, 7, 9, 12, 11, 10, 8, 5};
  uint p_16[12] = {6, 7, 9, 12, 14, 17, 16, 15, 13, 10, 8, 5};
  uint p_21[16] = {6, 7, 9, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5};

  uint p_11_16[14] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 15, 13, 10, 8, 5};
  uint p_11_21[18] = {6,  7,  9,  12, 11, 12, 14, 17, 19,
                      22, 21, 20, 18, 15, 13, 10, 8,  5};
  uint p_16_21[18] = {6,  7,  9,  12, 14, 17, 16, 17,19, 22,
                      21, 20, 18, 15, 13, 10, 8,  5};

  uint p_11_16_21[20] = {6,  7,  9,  12, 11, 12, 14, 17, 16, 17,
                         19, 22, 21, 20, 18, 15, 13, 10, 8,  5};
  //----------------------------------------------------------------------------

  vector<logistic_sim::Task> tasks;

  bool *init_agent;
  ProcessAgent *pa;


  void task_generator();


  // void compute_CF();

  void init(int argc, char **argv);
  void token_Callback(const logistic_sim::TokenConstPtr &msg);
  void task_Callback(const logistic_sim::TaskRequestConstPtr &msg);

private:
  // per l'inizializzazione e il token dei task
  ros::Subscriber sub_token;
  ros::Publisher  pub_token;

  ros::Subscriber sub_task; // quando un robot vuole un task
  ros::Publisher  pub_task; // pubblicazione dell'array (pop dal vettore di tasks)
};

} // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"