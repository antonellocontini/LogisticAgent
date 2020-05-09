#include "mapd.hpp"
#include <functional>

namespace mapd
{

using namespace std::placeholders;

mapd_search_tree::mapd_search_tree(const std::vector<std::vector<unsigned int> > &graph)
  : graph(graph), open(std::bind(&mapd_search_tree::cmp_function, this, _1, _2)), visited(), g(), f(), prev()
{

}


void mapd_search_tree::add_to_open(uint64_t state, unsigned int g_value, unsigned int f_value, uint64_t prev_state)
{
  g[state] = g_value;
  f[state] = f_value;
  prev[state] = prev_state;
  open[state] = f_value;
}


uint64_t mapd_search_tree::get_next_state() const
{
  return open.begin()->first;
}


void mapd_search_tree::pop_next_state()
{
  open.erase(open.begin());
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
  unsigned int lhs_f = f.find(lhs)->second, rhs_f = f.find(rhs)->second;
  unsigned int lhs_g = g.find(lhs)->second, rhs_g = g.find(rhs)->second;
  
  if (lhs_f < rhs_f)
  {
    return true;
  }
  else if (lhs_f == rhs_f && lhs_g > rhs_g)
  {
    return true;
  }
  return false;
}


} // namespace mapd