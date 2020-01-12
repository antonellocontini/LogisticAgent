#pragma once

#include "TaskPlanner.hpp"
// #include "boost/filesystem.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
// #include "message_types.hpp"

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

using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;

struct less_V
{
  inline bool operator()(const t_coalition &A, const t_coalition &B)
  {
    return A.second.V < B.second.V;
  }
};

void print_coalition(const t_coalition &coalition);

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

  void init(int argc, char **argv);
  void run();
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;
  virtual std::vector<logistic_sim::Path> path_partition(logistic_sim::Token &token, std::vector<logistic_sim::Mission> &missions) = 0;
  std::vector<logistic_sim::Mission> set_partition(const std::vector<logistic_sim::Mission> &ts);

  // definito per come ros vuole il tipo della funzione, chiamo quella della base class
  bool robot_ready(logistic_sim::RobotReady::Request &req, logistic_sim::RobotReady::Response &res)
  {
    return TaskPlanner::robot_ready(req, res);
  }
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

  // queste due funzioni scrivono/leggono una versione semplificata della struttura mission
  // in particolare andrebbero usate solo per missioni con singola destinazione
  // inoltre identificano la destinazione non con il vertice nel grafo ma con un indice
  // per garantire compatibilità con mappe diverse
  void write_simple_missions(std::ostream &os, const std::vector<logistic_sim::Mission> &mission);
  std::vector<logistic_sim::Mission> read_simple_missions(std::istream &is);

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

  int window_size = 4;
  std::list<std::vector<logistic_sim::Mission>> mission_windows;
  boost::mutex window_mutex;
  ros::Time last_goal_time;
  ros::Duration shutdown_timeout = ros::Duration(5 * 60.0), shutdown_warning = ros::Duration(4 * 60.0);
  bool warning_occured = false;
  logistic_sim::Token::_GOAL_STATUS_type last_goal_status;

  std::vector<ros::Subscriber> real_pos_sub, amcl_pos_sub;
  std::vector<nav_msgs::Odometry> last_real_pos;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> last_amcl_pos;
  std::vector<std::vector<double>> robot_pos_errors;

  // callback per leggere posizioni reali e misurate dei robot
  void real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot);
  void amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg, int id_robot);
};

}  // namespace onlinecentralizedtaskplanner

#include "impl/OnlineCentralizedTaskPlanner.i.hpp"