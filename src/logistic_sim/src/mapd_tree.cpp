#include "mapd.hpp"
#include <functional>
#include <numeric>

namespace mapd
{

using namespace std::placeholders;

mapd_search_tree::mapd_search_tree(const std::vector<std::vector<unsigned int> > &graph,
                                   const std::vector<unsigned int> &waypoints_number,
                                   const std::vector<unsigned int> &robot_ids)
  : graph(graph), open(std::bind(&mapd_search_tree::cmp_function, this, _1, _2)), visited(), g(), f(), prev(), waypoints_number(waypoints_number), robot_ids(robot_ids)
{

}


void mapd_search_tree::add_to_open(uint64_t state, unsigned int g_value, unsigned int f_value, uint64_t prev_state)
{
  prev[state] = prev_state;
  add_to_open(state, g_value, f_value);
}


void mapd_search_tree::add_to_open(uint64_t state, unsigned int g_value, unsigned int f_value)
{
  g[state] = g_value;
  f[state] = f_value;
  open[state] = f_value;
}


bool mapd_search_tree::is_open_empty() const
{
  return open.empty();
}


uint64_t mapd_search_tree::get_next_state() const
{
  return open.begin()->first;
}


void mapd_search_tree::pop_next_state()
{
  open.erase(open.begin());
}


uint64_t mapd_search_tree::open_size() const
{
  return open.size();
}


void mapd_search_tree::set_state_to_visited(uint64_t state)
{
  visited[state] = GRAY;
}


void mapd_search_tree::set_state_to_closed(uint64_t state)
{
  visited[state] = BLACK;
}


bool mapd_search_tree::is_state_visited(uint64_t state) const
{
  auto it = visited.find(state);
  if (it != visited.end() && it->second == GRAY)
  {
    return true;
  }
  return false;
}


bool mapd_search_tree::is_state_closed(uint64_t state) const
{
  auto it = visited.find(state);
  if (it != visited.end() && it->second == BLACK)
  {
    return true;
  }
  return false;
}


// can't search directly inside open because the cmp_function search
// inside f and g, which do not have the state if it's not present in open
// so, just check if is present in g
bool mapd_search_tree::is_state_in_queue(uint64_t state) const
{
  return g.count(state) != 0;
}


int mapd_search_tree::visited_state_f(uint64_t state) const
{
  auto it = f.find(state);
  if (it != f.end())
  {
    return it->second;
  }
  return -1;
}


int mapd_search_tree::visited_state_g(uint64_t state) const
{
  auto it = g.find(state);
  if (it != g.end())
  {
    return it->second;
  }
  return -1;
}


// void mapd_search_tree::set_prev_state(uint64_t state, uint64_t prev)
// {

// }


uint64_t mapd_search_tree::get_prev_state(uint64_t state) const
{
  auto it = prev.find(state);
  if (it != prev.end())
  {
    return it->second;
  }
  throw std::string("state is not present in tree!");
}


bool mapd_search_tree::cmp_function(uint64_t lhs, uint64_t rhs) const
{
  unsigned int lhs_f = f.find(lhs)->second,
               rhs_f = f.find(rhs)->second;
  unsigned int lhs_g = g.find(lhs)->second,
               rhs_g = g.find(rhs)->second;
  
  if (lhs_f < rhs_f)
  {
    return true;
  }
  else if (lhs_f == rhs_f)
  {
    mapd_state lhs_state(lhs, graph.size(), waypoints_number, robot_ids),
               rhs_state(rhs, graph.size(), waypoints_number, robot_ids);
    uint lhs_progress = 999, rhs_progress = 999;
    for (int i=0; i<waypoints_number.size(); i++)
    {
      if (lhs_state.waypoint_indices[i] < lhs_progress)
      {
        lhs_progress = lhs_state.waypoint_indices[i];
      }
      if (rhs_state.waypoint_indices[i] < rhs_progress)
      {
        rhs_progress = rhs_state.waypoint_indices[i];
      }
    }

    if (lhs_progress > rhs_progress)
    {
      return true;
    }
    else if (lhs_progress == rhs_progress && lhs_g > rhs_g)
    {
      return true;
    }
    // if (lhs_g > rhs_g)
    // {
    //   return true;
    // }
    // else if (lhs_g == rhs_g && lhs_progress > rhs_progress)
    // {
    //   return true;
    // }
  }
  return lhs < rhs;
}


const std::vector<std::vector<unsigned int> >& mapd_search_tree::get_graph() const
{
  return graph;
}


} // namespace mapd