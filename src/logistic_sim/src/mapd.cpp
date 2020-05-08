#include "mapd.hpp"

namespace mapd
{


mapd_state::mapd_state()
  : configuration(), waypoint_indices()
{

}


mapd_state::mapd_state(const std::vector<unsigned int> &configuration, const std::vector<unsigned int> &waypoint_indices)
  : configuration(configuration), waypoint_indices(waypoint_indices)
{

}


mapd_state::mapd_state(const mapd_state &s)
  : configuration(s.configuration), waypoint_indices(s.waypoint_indices)
{

}


mapd_state::mapd_state(unsigned int index, unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number)
  : configuration(waypoints_number.size()), waypoint_indices(waypoints_number.size())
{
  int robots_number = waypoints_number.size();

  for (int i=robots_number-1; i>=0; i--)
  {
    waypoint_indices[i] = index % waypoints_number[i];
    index /= waypoints_number[i];
  }

  for (int i=robots_number-1; i>=0; i--)
  {
    configuration[i] = index % vertices_number;
    index /= vertices_number;
  }
}


uint64_t mapd_state::get_index_notation(unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number)
{
  uint64_t index = 0;
  int robots_number = configuration.size();

  for (int i=0; i<robots_number; i++)
  {
    index *= vertices_number;
    index += configuration[i];
  }

  for (int i=0; i<robots_number; i++)
  {
    index *= waypoints_number[i];
    index += waypoint_indices[i];
  }

  return index;
}

}