#pragma once
#include "OnlineAgent.hpp"
#include "logistic_sim/UtilDPOP.h"
#include "logistic_sim/ValueDPOP.h"

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
  std::list<uint> active_waypoints;

  ros::ServiceServer dpop_util_msg_service, dpop_value_msg_service;
  bool dpop_util_message_handler(logistic_sim::UtilDPOP::Request &msg, logistic_sim::UtilDPOP::Response &res);
  bool dpop_value_message_handler(logistic_sim::ValueDPOP::Request &msg, logistic_sim::ValueDPOP::Response &res);

  struct pseudotree_node
  {
    uint parent;
    std::vector<uint> children, pseudo_children, pseudo_parents;
  };

  pseudotree_node tree_data;
};
}