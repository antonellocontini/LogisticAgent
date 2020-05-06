#pragma once
#include "OnlineTaskPlanner.hpp"

namespace onlinedcoptaskplanner
{
class OnlineDCOPTaskPlanner : public onlinetaskplanner::OnlineTaskPlanner
{
public:
  OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineDCOPTaskPlanner");
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;
};
}