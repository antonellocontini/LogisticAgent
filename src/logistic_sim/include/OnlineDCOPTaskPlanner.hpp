#pragma once
#include "OnlineTaskPlanner.hpp"
#include "logistic_sim/ChangeEdge.h"

namespace onlinedcoptaskplanner
{

struct edge_removal_plan
{
  uint from;
  uint to;
  uint timestep;
};

class OnlineDCOPTaskPlanner : public onlinetaskplanner::OnlineTaskPlanner
{
public:
  OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineDCOPTaskPlanner");

  void init(int argc, char **argv) override;
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

protected:
  std::list<edge_removal_plan> edge_list;

  bool first_missions_sent = false;
  uint first_valid_timestep = 0;

  // std::chrono::system_clock::time_point last_edge_removal;
  ros::Time last_edge_removal;

  ros::ServiceServer change_edge_service;
  void print_graph();   // for test
  void advertise_change_edge_service(ros::NodeHandle &nh) override;
  bool change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res);

  // edges that must be removed in the next token cycle
  std::vector<logistic_sim::Edge> removed_edges;
  boost::mutex edges_mutex; // used to avoid conflicts with the token thread and the main thread

  std::vector<bool> _check_conflict_free_impl(uint task_endpoint);
  bool check_conflict_free_property();

  // check if a configuration is good for recovery
  bool check_valid_recovery_configuration(const std::vector<uint> &configuration
                                        , const std::vector<uint> &robot_ids
                                        , const std::vector<std::vector<uint> > &waypoints
                                        , double* reachability_factor = nullptr);


  std::vector<std::vector<uint> > find_all_recovery_configs(const std::vector<std::vector<uint> > &waypoints, const std::vector<uint> &robot_ids);
  std::vector<uint> find_best_recovery_config(const std::vector<std::vector<uint> > &waypoints
                                            , const std::vector<uint> &robot_ids
                                            , const std::vector<uint> &current_config
                                            , const std::vector<std::vector<uint> > &other_paths);

  void _generate_near_configs_impl(std::vector<std::vector<uint> > &result, const std::vector<uint> &s, std::vector<uint> &temp_state, unsigned int robot_i = 0);
  std::vector<std::vector<uint> > generate_near_configs(const std::vector<uint> &config);
  std::vector<uint> local_search_recovery_config(const std::vector<std::vector<uint> > &waypoints
                                            , const std::vector<uint> &robot_ids
                                            , const std::vector<std::vector<uint> > &other_paths);

  void init_token(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void multi_agent_repair(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);

  std::vector<std::vector<unsigned int>> map_graph, fw_matrix;

  std::vector<std::vector<unsigned int> > build_graph();
  void build_fw_matrix();
};
}