#pragma once

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <map>
#include <functional>
#include <limits>

// library to handle mapd states for search problems
// assumes that graph vertices are identified with numbers starting with 0, no gaps

namespace mapd
{

const unsigned int WHITE = 0;
const unsigned int GRAY = 1;
const unsigned int BLACK = 2;

struct mapd_state
{
  mapd_state();
  mapd_state(const std::vector<unsigned int> &configuration, const std::vector<unsigned int> &waypoint_indices, const std::vector<unsigned int> &robot_ids);
  mapd_state(const mapd_state &s);
  // constructor from index notation
  mapd_state(unsigned int index, unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number, const std::vector<unsigned int> &robot_ids);

  std::vector<unsigned int> configuration;
  std::vector<unsigned int> waypoint_indices;
  std::vector<unsigned int> robot_ids;

  // produce an unique integer id, given the number of vertices in the graph and the number of waypoints per robot
  // this is useful to uniquely identify a state inside map and unordered_map data structures
  uint64_t get_index_notation(unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number) const;

  // use the graph in form of adjacency list and the waypoints to enumerate all the neighbours of the current state
  // does not check for conflicting states!!!
  std::vector<mapd_state> get_neigh(const std::vector<std::vector<unsigned int> > &gr, const std::vector<std::vector<unsigned int> > &waypoints) const;
  std::vector<uint64_t> get_neigh_index_notation(const std::vector<std::vector<unsigned int> > &gr, const std::vector<std::vector<unsigned int> > &waypoints) const;
  void _get_neigh_impl(const std::vector<std::vector<unsigned int> > &gr,
                        const std::vector<std::vector<unsigned int> > &waypoints,
                        std::vector<mapd_state> &result,
                        mapd_state &temp_state,
                        unsigned int robot_i) const;
};

// holds mapd search tree
class mapd_search_tree
{
public:
  mapd_search_tree(const std::vector<std::vector<unsigned int> > &graph);
  void add_to_open(uint64_t state, unsigned int g_value, unsigned int f_value, uint64_t prev_state);
  void add_to_open(uint64_t state, unsigned int g_value, unsigned int f_value);
  bool is_open_empty() const;
  uint64_t get_next_state() const;
  void pop_next_state();
  void set_state_to_visited(uint64_t state);
  void set_state_to_closed(uint64_t state);
  bool is_state_visited(uint64_t state) const;
  bool is_state_closed(uint64_t state) const;
  int visited_state_f(uint64_t state) const;
  int visited_state_g(uint64_t state) const;
  // void set_prev_state(uint64_t state, uint64_t prev);

  // this method throws an exception if prev does not exists
  uint64_t get_prev_state(uint64_t state) const;
  const std::vector<std::vector<unsigned int> >& get_graph();
protected:
  bool cmp_function(uint64_t lhs, uint64_t rhs) const;
  const std::vector<std::vector<unsigned int> > &graph;
  std::map<uint64_t, unsigned int, std::function<bool (uint64_t, uint64_t)> > open;  // map ordered by compare function
  std::unordered_map<uint64_t, unsigned int> visited, g, f; // keeps already visited states, tells if they are gray or black
  std::unordered_map<uint64_t, uint64_t> prev;  // keeps ancestors
};

struct max_cost_heuristic
{

std::vector<std::vector<unsigned int> > graph;
std::vector<std::vector<unsigned int> > fw;
std::vector<std::vector<unsigned int> > waypoints;
std::vector<unsigned int> waypoints_number;
std::vector<unsigned int> robot_ids;

max_cost_heuristic(const std::vector<std::vector<unsigned int> > &graph, const std::vector<std::vector<unsigned int> > &waypoints, const std::vector<unsigned int> &robot_ids)
  : graph(graph), fw(graph.size(), std::vector<unsigned int>(graph.size(), std::numeric_limits<unsigned int>::max())), waypoints(waypoints), waypoints_number(waypoints.size()), robot_ids(robot_ids)
{

// build waypoints_number
for (int i=0; i<waypoints.size(); i++)
{
  waypoints_number[i] = waypoints[i].size();
}

// build floyd-warshall matrix
unsigned int max = std::numeric_limits<unsigned int>::max();
unsigned int vertices = graph.size();
for (int i=0; i<vertices; i++)
{
  fw[i][i] = 0;
  for (int j=0; j<graph[i].size(); j++)
  {
    fw[i][j] = 1;
  }
}

for (int k=0; k<vertices; k++)
{
  for (int i=0; i<vertices; i++)
  {
    for (int j=0; j<vertices; j++)
    {
      if (fw[i][k] != max && fw[k][j] != max && fw[i][k] + fw[k][j] < fw[i][j])
      {
        fw[i][j] = fw[i][k] + fw[k][j];
      }
    }
  }
}

}

uint operator()(uint64_t state_index)
{
  uint vertices = graph.size();
  mapd_state s(state_index, vertices, waypoints_number, robot_ids);
  uint result = 0;
  uint agents = robot_ids.size();
  for (int i=0; i<agents; i++)
  {
    uint agent_cost = 0;
    uint curr_pos = s.configuration[i];
    uint next_pos;
    for (int j=s.waypoint_indices[i]; j<waypoints_number[i]; j++)
    {
      next_pos = waypoints[i][s.waypoint_indices[i]];
      agent_cost += fw[curr_pos][next_pos];
      curr_pos = next_pos;
    }

    result = std::max(result, agent_cost);
  }

  return result;
}
};

}
