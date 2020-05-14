#include "mapd.hpp"

namespace mapd
{


std::ostream& operator<<(std::ostream &out, const std::vector<uint> &v)
{
  for (const uint &x : v)
  {
    out << " " << x;
  }
  out << std::endl;
  return out;
}


std::ostream& operator<<(std::ostream& out, const mapd_state &s)
{
  out << "configuration:" << std::endl;
  for (const auto &x : s.configuration)
  {
    out << " " << x;
  }
  out << std::endl;
  out << "waypoint_indices:" << std::endl;
  for (const auto &x : s.waypoint_indices)
  {
    out << " " << x;
  }
  out << std::endl;
  out << "robot_ids:" << std::endl;
  for (const auto &x : s.robot_ids)
  {
    out << " " << x;
  }
  out << std::endl;
  return out;
}


mapd_state::mapd_state()
  : configuration(), waypoint_indices(), robot_ids()
{

}


mapd_state::mapd_state(const std::vector<unsigned int> &configuration, const std::vector<unsigned int> &waypoint_indices, const std::vector<unsigned int> &robot_ids)
  : configuration(configuration), waypoint_indices(waypoint_indices), robot_ids(robot_ids)
{

}


mapd_state::mapd_state(const mapd_state &s)
  : configuration(s.configuration), waypoint_indices(s.waypoint_indices), robot_ids(s.robot_ids)
{

}


mapd_state::mapd_state(unsigned int index, unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number, const std::vector<unsigned int> &robot_ids)
  : configuration(waypoints_number.size()), waypoint_indices(waypoints_number.size()), robot_ids(robot_ids)
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


uint64_t mapd_state::get_index_notation(unsigned int vertices_number, const std::vector<unsigned int> &waypoints_number) const
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

void mapd_state::_get_neigh_impl(const std::vector<std::vector<unsigned int> > &gr,
                                  const std::vector<std::vector<unsigned int> > &waypoints,
                                  std::vector<mapd_state> &result,
                                  mapd_state &temp_state,
                                  unsigned int robot_i) const
{
  int robots_number = configuration.size();
  uint robot_current_pos = configuration[robot_i];
  std::vector<unsigned int> near_vertices = {robot_current_pos};
  near_vertices.insert(near_vertices.end(), gr[robot_current_pos].begin(), gr[robot_current_pos].end());
  for (unsigned int v : near_vertices)
  {
    temp_state.configuration[robot_i] = v;
    unsigned int next_waypoint = waypoints[robot_i][waypoint_indices[robot_i]];
    if (v == next_waypoint && waypoint_indices[robot_i] < waypoints[robot_i].size() - 1)
    {
      temp_state.waypoint_indices[robot_i] = waypoint_indices[robot_i] + 1;
    }
    else
    {
      temp_state.waypoint_indices[robot_i] = waypoint_indices[robot_i];
    }
    
    if (robot_i == robots_number - 1)
    {
      result.push_back(temp_state);
    }
    else
    {
      _get_neigh_impl(gr, waypoints, result, temp_state, robot_i+1);
    }
    
  }
}


std::vector<mapd_state> mapd_state::get_neigh(const std::vector<std::vector<unsigned int> > &gr, const std::vector<std::vector<unsigned int> > &waypoints) const
{
  int robots_number = configuration.size();
  std::vector<mapd_state> result;
  mapd_state temp_state(*this);
  _get_neigh_impl(gr, waypoints, result, temp_state, 0);
  return result;
}


std::vector<uint64_t> mapd_state::get_neigh_index_notation(const std::vector<std::vector<unsigned int> > &gr, const std::vector<std::vector<unsigned int> > &waypoints) const
{
  std::vector<uint64_t> result;
  std::vector<unsigned int> waypoints_number;
  for (const auto &x : waypoints)
  {
    waypoints_number.push_back(x.size());
  }
  for (const mapd_state &x : get_neigh(gr, waypoints))
  {
    result.push_back(x.get_index_notation(gr.size(), waypoints_number));
  }

  return result;
}

}