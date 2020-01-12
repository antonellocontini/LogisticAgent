#pragma once

#include "OnlineCentralizedTaskPlanner.hpp"

using namespace onlinecentralizedtaskplanner;

namespace onlinegreedytaskplanner
{

class OnlineGreedyTaskPlanner : public onlinecentralizedtaskplanner::OnlineCentralizedTaskPlanner
{
public:
  OnlineGreedyTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineGreedyTaskPlanner");

  std::vector<logistic_sim::Path> path_partition(logistic_sim::Token &token, std::vector<logistic_sim::Mission> &missions) override;
};

}  // namespace onlinegreedytaskplanner

// #include "impl/OnlineGreedyTaskPlanner.i.hpp"