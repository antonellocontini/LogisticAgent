#include <vector>
#include <cstdint>

// library to handle mapd states for search problems
// assumes that graph vertices are identified with numbers starting with 0, no gaps

namespace mapd
{

struct mapd_state
{
  mapd_state();
  mapd_state(const std::vector<unsigned int> &configuration, const std::vector<unsigned int> &waypoint_indices);
  mapd_state(const mapd_state &s);
  // constructor from index notation
  mapd_state(unsigned int index, unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number);

  std::vector<unsigned int> configuration;
  std::vector<unsigned int> waypoint_indices;

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

}
