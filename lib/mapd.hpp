#pragma once

#include <vector>
#include <cstdint>
#include <unordered_map>
#include <map>
#include <functional>

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
  uint64_t get_prev_state(uint64_t state) const;
  const std::vector<std::vector<unsigned int> >& get_graph();
protected:
  bool cmp_function(uint64_t lhs, uint64_t rhs) const;
  const std::vector<std::vector<unsigned int> > &graph;
  std::map<uint64_t, unsigned int, std::function<bool (uint64_t, uint64_t)> > open;  // map ordered by compare function
  std::unordered_map<uint64_t, unsigned int> visited, g, f; // keeps already visited states, tells if they are gray or black
  std::unordered_map<uint64_t, uint64_t> prev;  // keeps ancestors
};

}
