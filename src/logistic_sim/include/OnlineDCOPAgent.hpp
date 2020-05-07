#pragma once
#include "OnlineAgent.hpp"

namespace onlinedcopagent
{
class OnlineDCOPAgent : public onlineagent::OnlineAgent
{
public:
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

protected:
  // override OnlineAgent methods to keep track of waypoint progress, needed for plan repair
  void token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token) override;
  void token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token) override;

  // book-keeping of the waypoints still to reach in the current plan
  std::list<uint> active_waypoints;
};
}