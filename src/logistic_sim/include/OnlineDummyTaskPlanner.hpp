#pragma once
#include "OnlineTaskPlanner.hpp"
#include "logistic_sim/NewPaths.h"

namespace onlinedummytaskplanner
{

struct edge_modification_plan
{
  uint from;
  uint to;
  uint timestep;
  int type; //1 == removal, 0 == addition

  const static int REMOVAL = 1;
  const static int ADDITION = 0;
};

class OnlineDummyTaskPlanner : public onlinetaskplanner::OnlineTaskPlanner
{
public:
  OnlineDummyTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineDummyTaskPlanner");

  void init(int argc, char **argv) override;
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

protected:
  bool first_missions_sent = false;
  uint first_valid_timestep = 0;

  std::vector<logistic_sim::Path> new_paths_vec;
  ros::ServiceServer new_paths_service;
  void advertise_new_paths_service(ros::NodeHandle &nh) override;
  bool new_paths(logistic_sim::NewPaths::Request &msg, logistic_sim::NewPaths::Response &res);


  void init_token(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
};
}