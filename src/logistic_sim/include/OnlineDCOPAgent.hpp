#pragma once
#include <memory>
#include <unordered_set>
#include "OnlineAgent.hpp"
//#include "logistic_sim/UtilDPOP.h"
//#include "logistic_sim/ValueDPOP.h"
#include "mapd.hpp"

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
                             logistic_sim::Token &token, std::vector<unsigned int> &first_leg,
                             std::vector<unsigned int> &last_leg);

  struct single_agent_node
  {
    uint vertex;
    uint timestep;
    uint f;
    uint waypoint;

    bool operator<(const single_agent_node &other) const
    {
      if (f < other.f)
      {
        return true;
      }
      else if (f == other.f)
      {
        if (timestep > other.timestep)
        {
          return true;
        }
        else if (timestep == other.timestep)
        {
          if (waypoint > other.waypoint)
          {
            return true;
          }
          else if (waypoint == other.waypoint)
          {
            return vertex < other.vertex;
          }
        }
      }
      return false;
    }

    bool operator==(const single_agent_node &other) const
    {
      if (vertex == other.vertex && timestep == other.timestep && waypoint == other.waypoint)
      {
        return true;
      }
      return false;
    }
  };

  struct sa_node_hash
  {
    std::size_t operator()(const single_agent_node &s) const
    {
      std::size_t h1 = std::hash<uint>{}(s.vertex);
      std::size_t h2 = std::hash<uint>{}(s.timestep);
      std::size_t h3 = std::hash<uint>{}(s.waypoint);
      return ((h1 ^ (h2 << 1)) >> 1) ^ (h3 << 1);
    }
  };

  uint calculate_h_value(const single_agent_node &n, const std::vector<unsigned int> &waypoints,
                         const std::vector<std::vector<unsigned int>> &min_hops_matrix);
  std::vector<unsigned int> spacetime_astar_dyn_mem(const std::vector<logistic_sim::Path> &other_paths,
                                                    const std::vector<std::vector<unsigned int>> &graph,
                                                    const std::vector<unsigned int> &waypoints,
                                                    const std::vector<std::vector<unsigned int>> &min_hops_matrix,
                                                    int start_time = 0, std::vector<unsigned int> *last_leg = nullptr,
                                                    std::vector<unsigned int> *first_leg = nullptr);

  // book-keeping of the waypoints still to reach in the current plan
  std::list<logistic_sim::Mission> assigned_missions;
  std::list<std::list<uint>> task_waypoints;
};

}  // namespace onlinedcopagent
