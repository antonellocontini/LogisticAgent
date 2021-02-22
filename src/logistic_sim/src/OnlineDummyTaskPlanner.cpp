#include <execinfo.h>
#include <signal.h>
#include <chrono>
#include <random>
#include <unordered_map>
#include "OnlineDummyTaskPlanner.hpp"
#include "boost/filesystem.hpp"
#include "get_graph.hpp"
#include "mapd.hpp"
#include "mapd_time_test.hpp"
#include "mapf.hpp"

#define EDGE_REMOVAL_TEST false

namespace onlinedummytaskplanner
{
std::stringstream log_ss;

std::string current_time_str()
{
  auto current_time = std::chrono::system_clock::now();
  std::time_t now_t = std::chrono::system_clock::to_time_t(current_time);
  char c_str[50];
  tm *timeinfo;
  timeinfo = localtime(&now_t);
  strftime(c_str, 50, "[%F %T]", timeinfo);
  return std::string(c_str);
}

std::string current_time_filename_str()
{
  auto current_time = std::chrono::system_clock::now();
  std::time_t now_t = std::chrono::system_clock::to_time_t(current_time);
  char c_str[50];
  tm *timeinfo;
  timeinfo = localtime(&now_t);
  strftime(c_str, 50, "%F-%T", timeinfo);
  return std::string(c_str);
}

OnlineDummyTaskPlanner::OnlineDummyTaskPlanner(ros::NodeHandle &nh_, const std::string &name)
  : OnlineTaskPlanner(nh_, name)
{
}

bool is_transition_valid(const mapd::mapd_state &from, const mapd::mapd_state &to,
                         const std::vector<std::vector<uint>> &other_paths, uint timestep)
{
  int robots_number = to.configuration.size();
  for (uint x : to.configuration)
  {
    // check vertex conflicts
    if (std::count(to.configuration.begin(), to.configuration.end(), x) > 1)
    {
      // ROS_DEBUG_STREAM("Internal vertex conflict");
      return false;
    }

    // check vertex conflicts with other paths
    for (const auto &p : other_paths)
    {
      if (p.size() > timestep + 1 && p[timestep + 1] == x)
      {
        // ROS_DEBUG_STREAM("Vertex conflict with other robot at timestep " << timestep+1);
        return false;
      }
      else if (p.size() <= timestep + 1 && p.back() == x)
      {
        // ROS_DEBUG_STREAM("Vertex conflict with other robot at timestep " << timestep+1);
        return false;
      }
    }
  }

  for (int i = 0; i < robots_number; i++)
  {
    // check swap conflicts within state
    for (int j = i; j < robots_number; j++)
    {
      if (to.configuration[i] == from.configuration[j] && to.configuration[j] == from.configuration[i])
      {
        // ROS_DEBUG_STREAM("Internal swap conflict");
        return false;
      }
    }

    // check swap conflicts with other paths
    for (const auto &p : other_paths)
    {
      if (p.size() > timestep + 1 && to.configuration[i] == p[timestep] && from.configuration[i] == p[timestep + 1])
      {
        // ROS_DEBUG_STREAM("Swap conflict with other robot from timestep " << timestep << " to timestep " <<
        // timestep+1);
        return false;
      }
    }
  }

  return true;
}

template <class T>
std::ostream &operator<<(std::ostream &out, const std::vector<std::vector<T>> &v)
{
  for (const auto &w : v)
  {
    for (const T &x : w)
    {
      out << " " << x;
    }
    out << " END" << std::endl;
  }
  return out;
}

template <class T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &v)
{
  for (const T &x : v)
  {
    out << " " << x;
  }
  out << std::endl;
  return out;
}


void OnlineDummyTaskPlanner::init(int argc, char **argv)
{
  OnlineTaskPlanner::init(argc, argv);
  std::string str_time = current_time_str();
  log_ss << str_time << "\n"
         << "\tmapname: " << mapname << "\n"
         << "\ttask_set_file: " << task_set_file << "\n";
}

void OnlineDummyTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{
  if (msg->ID_RECEIVER != TASK_PLANNER_ID)
    return;

  logistic_sim::Token token;
  token = *msg;
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;

  if (msg->INIT)
  {
    init_token(msg, token);
  }
  else
  {
    token.HEADER.seq += 1;

    // checking robot liveness
    if (std::equal(token.GOAL_STATUS.begin(), token.GOAL_STATUS.end(), last_goal_status.begin()))
    {
      auto delta = ros::Time::now() - last_goal_time;
      if (delta >= shutdown_timeout)
      {
        ROS_ERROR("Shutdown timeout has occured!!! Shutting down in 3 seconds");
        ros::Duration(3.0).sleep();
        ros::shutdown();
        int cmd_result = system("./stop_experiment.sh");
      }
      else if (delta >= shutdown_warning && !warning_occured)
      {
        ROS_WARN("Stuck robots detected, shutting down in 1 minute");
        warning_occured = true;
      }
    }
    else
    {
      last_goal_status = token.GOAL_STATUS;
      last_goal_time = ros::Time::now();
      if (warning_occured)
      {
        ROS_INFO("Timeout resetted");
        warning_occured = false;
      }
    }

    // checking conflicts on paths, this must be done
    // when robots are goal-synchronized
    bool eq = true;
    int v = token.GOAL_STATUS[0];
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      if (v != token.GOAL_STATUS[i])
      {
        eq = false;
        break;
      }
    }
    if (eq)
    {
      if (!new_paths_vec.empty())
      {
        if (new_paths_vec.size() == token.TRAILS.size())
        {
          token.TRAILS = new_paths_vec;
          token.ALLOCATE = true;
          ROS_DEBUG_STREAM("new_paths: " << new_paths_vec);
          ROS_DEBUG_STREAM("goal status: " << token.GOAL_STATUS);
        }
        new_paths_vec.clear();
      }
      std::vector<logistic_sim::Path> paths = token.TRAILS;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        paths[i].PATH.insert(paths[i].PATH.end(), token.HOME_TRAILS[i].PATH.begin(), token.HOME_TRAILS[i].PATH.end());
      }
      check_paths_conflicts(paths);
      // check_paths_conflicts(token.TRAILS); VECCHIA VERSIONE
    }

    if (token.SHUTDOWN)
    {
      ros::NodeHandle nh;
      nh.setParam("/simulation_running", "false");
      ros::shutdown();
      // int cmd_result = system("./stop_experiment.sh");
    }
  }

  pub_token.publish(token);
  ros::spinOnce();
}

void OnlineDummyTaskPlanner::init_token(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  token.HEADER.seq = 1;
  CAPACITY = msg->CAPACITY;
  token.INIT = false;
  token.END_SIMULATION = false;
  token.ALL_MISSIONS_INSERTED = false;
  token.NEW_HOMES = std::vector<int>(TEAM_SIZE, -1);
  if (!offline_mode)
  {
    ROS_DEBUG("online mode -- start time setted inside token init phase");
    start_time = ros::Time::now();
  }
  last_goal_time = ros::Time::now();
  last_goal_status = std::vector<unsigned int>(TEAM_SIZE, 0);
}

void OnlineDummyTaskPlanner::advertise_new_paths_service(ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM("Advertising new_paths service");
  new_paths_service = nh.advertiseService("new_paths", &OnlineDummyTaskPlanner::new_paths, this);
  if (!new_paths_service)
  {
    ROS_ERROR_STREAM("Can't create new_paths service");
  }
  else
  {
    ROS_INFO_STREAM("new_paths service advertised successfully");
  }
}

bool OnlineDummyTaskPlanner::new_paths(logistic_sim::NewPaths::Request &msg, logistic_sim::NewPaths::Response &res)
{
  new_paths_vec = msg.paths;
  res.result = true;
  return true;
}

}  // namespace onlinedcoptaskplanner

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinedummytaskplanner::OnlineDummyTaskPlanner ODTP(nh_);
  ODTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ODTP.run();
  sleep(3);
  int cmd_result = system("tmux kill-session -t log_sim");
  return 0;
}
