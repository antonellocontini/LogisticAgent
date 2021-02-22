#pragma once
#include <memory>
#include <unordered_set>
#include "OnlineAgent.hpp"
//#include "logistic_sim/UtilDPOP.h"
//#include "logistic_sim/ValueDPOP.h"
#include "mapd.hpp"

namespace onlinedummyagent
{
class OnlineDummyAgent : public onlineagent::OnlineAgent
{
public:
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;
  void init(int argc, char **argv) override;

protected:
  // override OnlineAgent methods to keep track of waypoint progress, needed for plan repair
  void token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token) override;

  // book-keeping of the waypoints still to reach in the current plan
  std::list<logistic_sim::Mission> assigned_missions;
  std::list<std::list<uint>> task_waypoints;
};

}  // namespace onlinedcopagent
