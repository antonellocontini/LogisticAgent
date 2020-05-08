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
  uint64_t get_index_notation(unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number);
};

}
