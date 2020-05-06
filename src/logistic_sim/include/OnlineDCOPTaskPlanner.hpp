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
  void print_graph();   // for test
  void advertise_change_edge_service(ros::NodeHandle &nh) override;
  bool change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res);

  // edges that must be removed in the next token cycle
  std::vector<logistic_sim::Edge> removed_edges;
  boost::mutex edges_mutex; // used to avoid conflicts with the token thread and the main thread
};
}