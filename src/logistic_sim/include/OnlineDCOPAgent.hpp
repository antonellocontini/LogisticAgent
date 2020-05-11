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
  std::list<uint> active_waypoints;

  ros::ServiceServer dpop_util_msg_service, dpop_value_msg_service;
  bool dpop_util_message_handler(logistic_sim::UtilDPOP::Request &msg, logistic_sim::UtilDPOP::Response &res);
  bool dpop_value_message_handler(logistic_sim::ValueDPOP::Request &msg, logistic_sim::ValueDPOP::Response &res);

  std::map<uint, ros::ServiceClient> dpop_value_children;
  ros::ServiceClient dpop_util_parent;

  struct pseudotree_node
  {
    int parent;
    std::vector<uint> children, pseudo_children, pseudo_parents;

    pseudotree_node()
      : parent(-1), children(), pseudo_children(), pseudo_parents()
    {

    }
  };

  std::unique_ptr<pseudotree_node> tree_data;
  std::unique_ptr<mapd::mapd_search_tree> search_tree;

  bool is_dcop_root();
  int search_dcop_root_agent_id(const logistic_sim::Token &token);

  bool is_dcop_leaf();
  bool done_util_phase = false;
};
}