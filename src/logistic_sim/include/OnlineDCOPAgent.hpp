#pragma once
#include "OnlineAgent.hpp"
#include "logistic_sim/UtilDPOP.h"
#include "logistic_sim/ValueDPOP.h"
#include "mapd.hpp"
#include <memory>

namespace onlinedcopagent
{
class OnlineDCOPAgent : public onlineagent::OnlineAgent
{
public:
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;
  void init(int argc, char **argv) override;

protected:
  // override OnlineAgent methods to keep track of waypoint progress, needed for plan repair
  void token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token) override;
  void token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token) override;
  void plan_and_update_token(const std::vector<uint> &waypoints, std::vector<logistic_sim::Path> &robot_paths,
                                            logistic_sim::Token &token, std::vector<unsigned int> &first_leg, std::vector<unsigned int> &last_leg);

  // book-keeping of the waypoints still to reach in the current plan
  std::list<logistic_sim::Mission> assigned_missions;
  std::list<std::list<uint> > task_waypoints;
};
}