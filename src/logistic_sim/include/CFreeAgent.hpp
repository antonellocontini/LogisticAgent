#pragma once
#include "DistrAgent.hpp"

namespace cfreeagent
{
using namespace distragent;

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

class CFreeAgent : public DistrAgent
{
protected:
  bool path_calculated = false;
  std::vector<logistic_sim::Mission> missions;
  int home_count = 0, home_steps = -1;
  unsigned int ****prev_paths;
  unsigned int ***path_sizes;
  unsigned int ***visited;
  st_location *queue;
  const unsigned int WHITE = 0, GRAY = 1, BLACK = 2, MAX_TIME = 150U, MAX_WAYPOINTS = 64U;

  void allocate_memory()
  {
    std::cout << "allocating memory..." << std::endl;
    std::cout << dimension << " " << MAX_TIME << std::endl;
    prev_paths = new unsigned int ***[dimension];
    path_sizes = new unsigned int **[dimension];
    for (unsigned int i = 0; i < dimension; i++)
    {
      prev_paths[i] = new unsigned int **[MAX_TIME];
      path_sizes[i] = new unsigned int *[MAX_TIME];
      for (unsigned int j = 0; j < MAX_TIME; j++)
      {
        prev_paths[i][j] = new unsigned int *[MAX_WAYPOINTS];
        path_sizes[i][j] = new unsigned int[MAX_WAYPOINTS];
        for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
        {
          prev_paths[i][j][k] = new unsigned int[MAX_TIME];
        }
      }
    }

    visited = new unsigned int **[dimension];
    for (unsigned int i = 0; i < dimension; i++)
    {
      visited[i] = new unsigned int *[MAX_TIME];
      for (unsigned int j = 0; j < MAX_TIME; j++)
      {
        visited[i][j] = new unsigned int[MAX_WAYPOINTS];
      }
    }

    queue = new st_location[MAX_TIME * MAX_TIME];
  }
public:
  
  ~CFreeAgent()
  {
    for (unsigned int i = 0; i < dimension; i++)
    {
      for (unsigned int j = 0; j < MAX_TIME; j++)
      {
        for (unsigned int k = 0; k < MAX_TIME; k++)
        {
          delete[] prev_paths[i][k][j];
        }
        delete[] prev_paths[i][j];
        delete[] path_sizes[i][j];
        delete[] visited[i][j];
      }

      delete[] prev_paths[i];
      delete[] path_sizes[i];
      delete[] visited[i];
    }
    delete[] prev_paths;
    delete[] path_sizes;
    delete[] visited;
    delete[] queue;
  }
  bool token_check_pt(std::vector<uint> &my_path, std::vector<logistic_sim::Path> &other_paths, uint ID_ROBOT,
                      int *id_vertex_stuck);
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;
  std::vector<unsigned int> token_dijkstra(const std::vector<uint> &waypoints,
                                           std::vector<logistic_sim::Path> &other_paths,
                                           const std::vector<bool> &still_robots = std::vector<bool>());
  std::vector<unsigned int> spacetime_dijkstra(const std::vector<std::vector<unsigned int> > &other_paths,
                                               const std::vector<std::vector<unsigned int> > &graph, unsigned int size,
                                               const std::vector<unsigned int> &waypoints,
                                               const std::vector<bool> &still_robots);
};
}  // namespace cfreeagent

#include "impl/CFreeAgent.i.hpp"