#pragma once
#include "CFreeAgent.hpp"

namespace onlineagent
{
bool astar_cmp_function(const std::vector<std::vector<unsigned int>> &min_hops_matrix,
                        const std::vector<unsigned int> &waypoints, const st_location &lhs, const st_location &rhs)
{
  // uso la min_hops_matrix come euristica sulla lunghezza del percorso rimanente
  unsigned int lhs_h = min_hops_matrix[lhs.vertex][waypoints[lhs.waypoint]];
  unsigned int rhs_h = min_hops_matrix[rhs.vertex][waypoints[rhs.waypoint]];
  // TEST
  // std::cout << "[DEBUG]\tVertex: " << rhs.vertex << "\tTime: " << rhs.time
  //           << "\tWaypoint: " << waypoints[rhs.waypoint]
  //           << "\tf-value: " << rhs.time + rhs_h << std::endl;
  if (lhs.time + lhs_h < rhs.time + rhs_h)
  {
    return true;
  }
  // else if (lhs.time + lhs_h == rhs.time + rhs_h)
  // {
  //   if (lhs.waypoint > rhs.waypoint)
  //   {
  //     return true;
  //   }
  // }

  return false;
}

class OnlineAgent : public cfreeagent::CFreeAgent
{
public:
  void init(int argc, char **argv) override;
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

  // template versions of the dijkstra functions
  // T is any callable type that returns a bool and takes two st_location objects
  template <class T = bool(const st_location &, const st_location &)>
  std::vector<unsigned int> spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                               const std::vector<std::vector<unsigned int>> &graph,
                                               const std::vector<unsigned int> &waypoints, int start_time = 0,
                                               std::vector<unsigned int> *last_leg = nullptr,
                                               std::vector<unsigned int> *first_leg = nullptr,
                                               T *cmp_function = nullptr);

  template <class T = bool(const st_location &, const st_location &)>
  unsigned int insertion_sort(st_location *queue, unsigned int size, st_location loc, T *cmp_function = nullptr);

protected:
  // adjacency list of the graph, contains only neighbour of vertices, without edge lengths
  std::vector<std::vector<unsigned int>> map_graph, min_hops_matrix;
  bool still = true;

  std::vector<std::vector<unsigned int>> calculate_min_hops_matrix();

  void token_simple_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void token_simple_allocation(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void token_simple_planning(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);

  void token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
};

}  // namespace onlineagent

#include "impl/OnlineAgent.i.hpp"