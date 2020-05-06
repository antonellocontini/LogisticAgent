#pragma once
// #include <color_cout.hpp> //lib
// #include <partition.hpp>
#include <algorithm>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iostream>
#include <iterator>
#include <queue>
#include <set>
#include <sstream>
#include <sstream>
#include <stdexcept>
#include <stdexcept>
#include <vector>

#include <boost/thread.hpp>

#include <ros/package.h>  //to get pkg path
#include <ros/ros.h>
#include <sys/stat.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <logistic_sim/Mission.h>
#include <logistic_sim/Token.h>

#include "message_types.hpp"
#include "struct_vertex.hpp"
// #include "partition.hpp"
// #include "get_graph.hpp"
#include "color_cout.hpp"

#include "logistic_sim/RobotReady.h"
#include "logistic_sim/AddMissions.h"

namespace taskplanner
{
const std::string PS_path = ros::package::getPath("logistic_sim");

using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;

struct less_V
{
  inline bool operator()(const t_coalition &A, const t_coalition &B)
  {
    return A.second.V < B.second.V;
  }
};

void print_coalition(const t_coalition &coalition);

// dati relativi al singolo robot
struct MonitorData
{
  float tot_distance = 0.0f;
  int interference_num = 0;
  int completed_missions = 0;
  int completed_tasks = 0;
  float total_time = 0.0f;
  // campi ausiliari
  int last_dst = -1;
};

ostream &operator<<(ostream &os, const MonitorData &md);

ostream &operator<<(ostream &os, const vector<MonitorData> &v);

// override operatori per lettura e scrittura missioni su file
ostream &operator<<(ostream &os, const logistic_sim::Mission &m);
ostream &operator<<(ostream &os, const vector<logistic_sim::Mission> &v);

istream &operator>>(istream &is, logistic_sim::Mission &m);
istream &operator>>(istream &is, vector<logistic_sim::Mission> &v);

// confronto per test
bool operator==(const logistic_sim::Mission &lhs, const logistic_sim::Mission &rhs);
bool operator!=(const logistic_sim::Mission &lhs, const logistic_sim::Mission &rhs);

// override operatori per lettura e scrittura percorsi su file
ostream &operator<<(ostream &os, const logistic_sim::Path &p);
ostream &operator<<(ostream &os, const vector<logistic_sim::Path> &v);

istream &operator>>(istream &is, logistic_sim::Path &p);
istream &operator>>(istream &is, vector<logistic_sim::Path> &v);

class TaskPlanner
{
public:
  TaskPlanner(ros::NodeHandle &nh_, const std::string &name = "TaskPlanner");
  ~TaskPlanner(){};

  vertex *vertex_web;
  uint dimension;
  string mapname;

  std::string ALGORITHM;
  std::string GENERATION;
  uint TEAM_CAPACITY = 0;
  std::vector<uint> CAPACITY;
  uint TEAM_SIZE = 0;
  uint nTask = 0;
  uint id = 0;

  vector<logistic_sim::Mission> missions;

  vector<MonitorData> robots_data;
  bool first_round = true;
  int num_robots, robots_ready_count = 0;
  vector<bool> robots_ready_status;

  ros::Time start_time;

  // map_homes is currently not used, the home locations depend on the configuration
  // currently they are found by querying the robots about their initial position and
  // finding the nearest vertex
  std::map<std::string, std::vector<uint>> map_homes = { { "model6", { 0, 1, 2, 3, 4, 5 } },
                                                         { "grid", { 1, 2, 3, 21, 22, 23 } },
                                                         { { "icelab" }, { 0, 1, 2, 26, 27, 28 } },
                                                         { "model5", { 0, 1, 2, 25, 26, 27 } } };
  uint src_vertex;
  std::vector<uint> dst_vertex;
  std::vector<uint> home_vertex;
  std::vector<std::vector<uint>> paths;

  std::string task_set_file;

  int compute_cost_of_route(std::vector<uint> &route);

  void missions_generator(std::string &gen_type);

  void u_missions_generator();
  void nu_missions_generator();

  void random_mission(uint n_missions);
  void write_single_mission(ostream &os, const logistic_sim::Mission &m);
  void write_missions(ostream &os, const vector<logistic_sim::Mission> &v);
  logistic_sim::Mission read_single_mission(istream &is);
  vector<logistic_sim::Mission> read_missions(istream &is);

  int my_random(int n);

  logistic_sim::Mission create_mission(uint type, int id);
  // insert a window that will go straight to allocation phase - no aggregation
  void insert_mission_window(std::vector<logistic_sim::Mission> &window);

  virtual std::vector<logistic_sim::Mission> set_partition(const std::vector<logistic_sim::Mission> &ts);
  virtual std::vector<logistic_sim::Path> path_partition(logistic_sim::Token &token);

  virtual void init(int argc, char **argv);
  void run();

  virtual void token_callback(const logistic_sim::TokenConstPtr &msg) = 0;

  bool robot_ready(logistic_sim::RobotReady::Request &req, logistic_sim::RobotReady::Response &res);

  virtual void allocate_memory() = 0;

protected:
  // per l'inizializzazione e il token dei task
  ros::Subscriber sub_token;
  ros::Publisher pub_token;
  std::string name;

  std::list<std::vector<logistic_sim::Mission>> mission_windows;
  boost::mutex window_mutex;

  // temporaneo
  std::ofstream times_file;

  void write_missions_on_file(std::string filename = "");

  // methods needed for initialization
  void read_cmdline_parameters(int argc, char **argv);
  void set_map_endpoints(ros::NodeHandle &nh);
  void calculate_aggregation_paths();
  virtual void build_map_graph();

  ros::ServiceServer robot_ready_service, add_missions_service;
  void advertise_robot_ready_service(ros::NodeHandle &nh);
  void advertise_add_missions_service(ros::NodeHandle &nh);
  virtual void advertise_change_edge_service(ros::NodeHandle &nh);  // empty implementation, used by DCOPTaskPlanner
  bool add_missions(logistic_sim::AddMissions::Request &msg, logistic_sim::AddMissions::Response &res);

  std::vector<ros::Subscriber> real_pos_sub, amcl_pos_sub;
  std::vector<nav_msgs::Odometry> last_real_pos;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> last_amcl_pos;
  std::vector<std::vector<double>> robot_pos_errors;
  // callback per leggere posizioni reali e misurate dei robot
  void real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot);
  void amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg, int id_robot);
  void initialize_amcl_callbacks(ros::NodeHandle &nh);

  // queste due funzioni scrivono/leggono una versione semplificata della struttura mission
  // in particolare andrebbero usate solo per missioni con singola destinazione
  // inoltre identificano la destinazione non con il vertice nel grafo ma con un indice
  // per garantire compatibilità con mappe diverse
  void write_simple_missions(std::ostream &os, const std::vector<logistic_sim::Mission> &mission);
  std::vector<logistic_sim::Mission> read_simple_missions(std::istream &is);
  void generate_missions();
  void initialize_stats_structure();
  void open_times_file();
  void wait_agents();
  void initialize_token();

  bool offline_mode;
  int window_size;
  void detect_offline_mode();
};

}  // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"