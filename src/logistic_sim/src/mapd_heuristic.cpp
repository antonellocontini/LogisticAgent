#include "mapd.hpp"
#include <iomanip>

namespace mapd
{


std::ostream& operator<<(std::ostream &out, const max_cost_heuristic &h)
{
  unsigned int max = std::numeric_limits<unsigned int>::max();
  for (int i=0; i<h.fw.size(); i++)
  {
    for (int j=0; j<h.fw[i].size(); j++)
    {
      if (h.fw[i][j] == max)
      {
        out << " inf";
      }
      else
      {
        out << std::setw(4) << h.fw[i][j];
      }
      
    }
    out << std::endl;
  }
  return out;
}


max_cost_heuristic::max_cost_heuristic(const std::vector<std::vector<unsigned int> > &graph,
                   const std::vector<std::vector<unsigned int> > &waypoints, const std::vector<unsigned int> &robot_ids)
  : graph(graph)
  , fw(graph.size(), std::vector<unsigned int>(graph.size(), std::numeric_limits<unsigned int>::max()))
  , waypoints(waypoints)
  , waypoints_number(waypoints.size())
  , robot_ids(robot_ids)
{
  // build waypoints_number
  for (int i = 0; i < waypoints.size(); i++)
  {
    waypoints_number[i] = waypoints[i].size();
  }

  // build floyd-warshall matrix
  unsigned int max = std::numeric_limits<unsigned int>::max();
  unsigned int vertices = graph.size();
  for (int i = 0; i < vertices; i++)
  {
    fw[i][i] = 0;
    for (int j = 0; j < graph[i].size(); j++)
    {
      fw[i][graph[i][j]] = 1;
    }
  }

  for (int k = 0; k < vertices; k++)
  {
    for (int i = 0; i < vertices; i++)
    {
      for (int j = 0; j < vertices; j++)
      {
        if (fw[i][k] != max && fw[k][j] != max && fw[i][k] + fw[k][j] < fw[i][j])
        {
          fw[i][j] = fw[i][k] + fw[k][j];
        }
      }
    }
  }
}

uint max_cost_heuristic::operator()(uint64_t state_index)
{
  uint vertices = graph.size();
  mapd_state s(state_index, vertices, waypoints_number, robot_ids);
  uint result = 0;
  uint agents = robot_ids.size();
  for (int i = 0; i < agents; i++)
  {
    uint agent_cost = 0;
    uint curr_pos = s.configuration[i];
    uint next_pos;
    // std::cout << waypoints_number[i] << std::endl;
    for (int j = s.waypoint_indices[i]; j < waypoints_number[i]; j++)
    {
      next_pos = waypoints[i][j];
      agent_cost += fw[curr_pos][next_pos];
      // std::cout << "\tfrom " << curr_pos << " to " << next_pos << ": " << fw[curr_pos][next_pos] << std::endl;
      curr_pos = next_pos;
    }
    // std::cout << std::endl;

    result = std::max(result, agent_cost);
    // getc(stdin);
  }

  return result;
}

}  // namespace mapd