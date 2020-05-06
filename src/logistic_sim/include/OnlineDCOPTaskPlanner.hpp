#pragma once
#include "OnlineTaskPlanner.hpp"
#include "logistic_sim/ChangeEdge.h"

namespace onlinedcoptaskplanner
{
class OnlineDCOPTaskPlanner : public onlinetaskplanner::OnlineTaskPlanner
{
public:
  OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineDCOPTaskPlanner");
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

protected:

  ros::ServiceServer change_edge_service;
  void advertise_change_edge_service(ros::NodeHandle &nh);
  bool change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res);
};
}