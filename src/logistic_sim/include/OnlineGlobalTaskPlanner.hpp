#pragma once

#include "OnlineCentralizedTaskPlanner.hpp"

using namespace onlinecentralizedtaskplanner;

namespace onlineglobaltaskplanner
{

class OnlineGlobalTaskPlanner : public onlinecentralizedtaskplanner::OnlineCentralizedTaskPlanner
{
public:
  OnlineGlobalTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineGlobalTaskPlanner");

  std::vector<logistic_sim::Path> path_partition(logistic_sim::Token &token, std::vector<logistic_sim::Mission> &missions) override;
};

}  // namespace onlineglobaltaskplanner

// #include "impl/OnlineGlobalTaskPlanner.i.hpp"