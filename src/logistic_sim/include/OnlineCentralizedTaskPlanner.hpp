#pragma once

#include "TaskPlanner.hpp"
#include <boost/thread.hpp>

namespace onlinecentralizedtaskplanner
{
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

bool astar_cmp_function(const std::vector<std::vector<unsigned int>> &min_hops_matrix,
                        const std::vector<unsigned int> &waypoints, const st_location &lhs, const st_location &rhs);

const std::string PS_path = ros::package::getPath("logistic_sim");

class OnlineCentralizedTaskPlanner : public taskplanner::TaskPlanner
{
public:
  OnlineCentralizedTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineCentralizedTaskPlanner");
  ~OnlineCentralizedTaskPlanner()
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

  void token_callback(const logistic_sim::TokenConstPtr &msg) override;
  virtual std::vector<logistic_sim::Path> path_partition(logistic_sim::Token &token, std::vector<logistic_sim::Mission> &missions) = 0;

  void build_map_graph() override;

  bool check_paths_conflicts(const std::vector<logistic_sim::Path> &paths, bool print = true);

  // template versions of the dijkstra functions
  // T is any callable type that returns a bool and takes two st_location objects
  template <class T = bool(const st_location &, const st_location &)>
  std::vector<unsigned int> spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                               const std::vector<std::vector<unsigned int>> &graph,
                                               const std::vector<unsigned int> &waypoints, int ID_ROBOT,
                                               int start_time = 0, std::vector<unsigned int> *last_leg = nullptr,
                                               std::vector<unsigned int> *first_leg = nullptr,
                                               T *cmp_function = nullptr);

  template <class T = bool(const st_location &, const st_location &)>
  unsigned int insertion_sort(st_location *queue, unsigned int size, st_location loc, T *cmp_function = nullptr);

protected:
  // adjacency list of the graph, contains only neighbour of vertices, without edge lengths
  std::vector<std::vector<unsigned int>> map_graph, min_hops_matrix;
  std::vector<std::vector<unsigned int>> calculate_min_hops_matrix();
  std::vector<logistic_sim::Path> trails, home_trails;

  // astar fields
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

  bool offline_mode = false;
  ros::Time last_goal_time;
  ros::Duration shutdown_timeout = ros::Duration(5 * 60.0), shutdown_warning = ros::Duration(4 * 60.0);
  bool warning_occured = false;
  logistic_sim::Token::_GOAL_STATUS_type last_goal_status;
};

}  // namespace onlinecentralizedtaskplanner

#include "impl/OnlineCentralizedTaskPlanner.i.hpp"