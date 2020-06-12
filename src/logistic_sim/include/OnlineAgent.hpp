#pragma once
#include "Agent.hpp"

namespace onlineagent
{
// using namespace cfreeagent;
struct st_location
{
  unsigned int vertex;
  unsigned int time;
  unsigned int waypoint;

  st_location(unsigned int vertex = 0, unsigned int time = 0, unsigned int waypoint = 0)
    : vertex(vertex), time(time), waypoint(waypoint)
  {
  }
  st_location(const st_location &ref) : vertex(ref.vertex), time(ref.time), waypoint(ref.waypoint)
  {
  }
  bool operator<(const st_location &loc) const
  {
    if (time < loc.time)
    {
      return true;
    }
    else if (time == loc.time)
    {
      if (waypoint > loc.waypoint)
      {
        return true;
      }
    }
    return false;
  }
};

bool astar_cmp_function(const std::vector<std::vector<unsigned int>> &min_hops_matrix,
                        const std::vector<unsigned int> &waypoints, const st_location &lhs, const st_location &rhs);

class OnlineAgent : public agent::Agent
{
public:
  ~OnlineAgent();

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
  std::vector<logistic_sim::Mission> missions;
  unsigned int ****prev_paths;
  unsigned int ***path_sizes;
  unsigned int ***visited;
  st_location *queue;
  const unsigned int WHITE = 0, GRAY = 1, BLACK = 2, MAX_TIME = 300U, MAX_WAYPOINTS = 64U;
  void allocate_memory();

  // update graph and min hop matrix from the basic representation
  void update_graph();

  // adjacency list of the graph, contains only neighbour of vertices, without edge lengths
  std::vector<std::vector<unsigned int>> map_graph, min_hops_matrix;
  bool still = true;

  std::vector<std::vector<unsigned int> > build_graph();
  std::vector<std::vector<unsigned int> > calculate_min_hops_matrix();

  void token_simple_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void token_simple_allocation(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  void token_simple_planning(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);

  virtual void token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
  virtual void token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
};

}  // namespace onlineagent

#include "impl/OnlineAgent.i.hpp"