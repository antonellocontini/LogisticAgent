#include <execinfo.h>
#include <signal.h>
#include <chrono>
#include <random>
#include <unordered_map>
#include "OnlineDCOPTaskPlanner.hpp"
#include "boost/filesystem.hpp"
#include "get_graph.hpp"
#include "mapd.hpp"
#include "mapd_time_test.hpp"
#include "mapf.hpp"

#define EDGE_REMOVAL_TEST false

namespace onlinedcoptaskplanner
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

/*!
  Initializes the random number generator for the local repair
  currently the seed is fixed for reproducibility purpose
*/
OnlineDCOPTaskPlanner::OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name)
  : OnlineTaskPlanner(nh_, name)
{
  constexpr uint seed = 5;
  gen = std::mt19937(seed);
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

template <class T = uint(uint64_t)>
void astar_search_function(const std::vector<std::vector<uint>> &waypoints, const mapd::mapd_state &is,
                           const std::vector<std::vector<uint>> &graph, const std::vector<uint> &robot_ids, T *h_func,
                           const std::vector<std::vector<uint>> &other_paths,
                           std::vector<std::vector<uint>> *result_task_paths,
                           std::vector<std::vector<uint>> *result_home_paths, const std::vector<bool> &going_home)
{
  ROS_DEBUG_STREAM("waypoints: " << std::endl << waypoints);
  ROS_DEBUG_STREAM("robot_ids: " << std::endl << robot_ids);
  ROS_DEBUG_STREAM("other_paths: " << std::endl << other_paths);
  int robot_number = robot_ids.size();
  std::vector<uint> waypoints_number;
  for (const auto &x : waypoints)
  {
    waypoints_number.push_back(x.size());
  }
  ROS_DEBUG_STREAM("instantiate tree");
  mapd::mapd_search_tree tree(graph, waypoints_number, robot_ids);
  int vertices_number = graph.size();
  uint64_t is_index = is.get_index_notation(vertices_number, waypoints_number);
  ROS_DEBUG_STREAM("initial state index notation: " << is_index);
  // ROS_DEBUG_STREAM("heuristic function: " << *h_func);
  // std::ofstream test("test_heuristic.txt");
  // test << *h_func;
  // test.close();
  uint h_value = (*h_func)(is_index);
  ROS_DEBUG_STREAM("initial state h-value: " << h_value);
  ROS_DEBUG_STREAM("add initial state to tree");
  tree.add_to_open(is_index, 0, h_value);
  ROS_DEBUG_STREAM("initial state: " << std::endl << is);
  ROS_DEBUG_STREAM("Starting state exploration...");
  uint64_t count = 0, closed_count = 0;
  auto start = std::chrono::system_clock::now();

  // test final config
  std::vector<uint> final_config;
  for (int i = 0; i < robot_number; i++)
  {
    final_config.push_back(waypoints[i].back());
  }
  ROS_DEBUG_STREAM("final config: " << final_config);
  ROS_DEBUG_STREAM("robot_number: " << robot_number);

  // test reachability
  std::vector<std::vector<std::vector<bool>>> reachability(graph.size(), std::vector<std::vector<bool>>(robot_number));
  for (int j = 0; j < graph.size(); j++)
  {
    for (int i = 0; i < robot_number; i++)
    {
      reachability[j][i] = std::vector<bool>(waypoints_number[i] + 1, false);
    }
  }

  while (!tree.is_open_empty())
  {
    // timer to show statistics
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    if (elapsed.count() >= 10.0)
    {
      ROS_DEBUG_STREAM("Visited states: " << count);
      ROS_DEBUG_STREAM("Open queue size: " << tree.open_size());
      start = std::chrono::system_clock::now();
    }
    count++;
    uint64_t s_index = tree.get_next_state();
    // ROS_DEBUG_STREAM("VISIT: " << s_index);
    mapd::mapd_state s(s_index, vertices_number, waypoints_number, robot_ids);
    tree.set_state_to_closed(s_index);
    tree.pop_next_state();
    closed_count++;

    // check if this is a final state
    bool is_final = true;
    for (int i = 0; i < robot_number; i++)
    {
      // if a robot hasn't completed all its waypoints or is not on the final vertex
      // then this is not the final configuration
      if (s.waypoint_indices[i] < waypoints_number[i] || s.configuration[i] != waypoints[i].back())
      {
        is_final = false;
        break;
      }
    }

    for (int i = 0; i < robot_number; i++)
    {
      // update reachability
      uint v = s.configuration[i];
      reachability[v][i][s.waypoint_indices[i]] = true;
    }

    if (s.configuration == final_config)
    {
      ROS_DEBUG_STREAM(closed_count << "\tfinal config test - final config reached on these waypoints: "
                                    << s.waypoint_indices << "\tconfiguration: " << s.configuration
                                    << "\tfinal config: " << final_config);
    }

    // if (s.waypoint_indices == final_waypoints)
    // {
    //   ROS_DEBUG_STREAM(closed_count << "\tfinal waypoints test - last waypoints reached in this configuration " <<
    //   s.configuration);
    // }

    // ROS_DEBUG_STREAM(closed_count << "\t" << s.waypoint_indices);

    if (is_final)
    {
      ROS_DEBUG_STREAM("found solution!");
      // reconstruct path
      std::vector<std::vector<uint>> task_path(robot_number), home_path(robot_number);
      try
      {
        while (true)
        {
          // ROS_DEBUG_STREAM("final path: " << s.configuration);
          for (int i = 0; i < robot_number; i++)
          {
            // the last condition is to make sure that the second last waypoint is inserted in the task path, not the
            // home path
            if (going_home[i] && s.waypoint_indices[i] == waypoints_number[i] - 1 &&
                s.configuration[i] != waypoints[i][s.waypoint_indices[i] - 1])
            {
              home_path[i].emplace(home_path[i].begin(), s.configuration[i]);
            }
            else
            {
              task_path[i].emplace(task_path[i].begin(), s.configuration[i]);
            }
          }
          s = mapd::mapd_state(tree.get_prev_state(s_index), vertices_number, waypoints_number, robot_ids);
          s_index = s.get_index_notation(vertices_number, waypoints_number);
        }
      }
      catch (const std::string &e)
      {
        // reached initial state
        ROS_DEBUG_STREAM("initial h-value: " << h_value);
        ROS_DEBUG_STREAM("path length: " << task_path[0].size() + home_path[0].size());
        ROS_DEBUG_STREAM("returning complete paths!");
        *result_task_paths = task_path;
        *result_home_paths = home_path;
        return;
      }
    }

    uint s_g_value = tree.visited_state_g(s_index);
    std::vector<mapd::mapd_state> neigh_list = s.get_neigh(graph, waypoints);
    // if (neigh_list.empty())
    // {
    //   ROS_DEBUG_STREAM("Warning! Neighbour list is empty");
    // }
    // ROS_DEBUG_STREAM("Neighbours #: " << neigh_list.size());
    for (const mapd::mapd_state &x : neigh_list)
    {
      uint64_t x_index = x.get_index_notation(vertices_number, waypoints_number);
      if (x_index != s_index)
      {
        if (is_transition_valid(s, x, other_paths, s_g_value))
        {
          // check that neighbour state is new or reached through a better path
          int neigh_g_value = tree.visited_state_g(x_index);
          if (neigh_g_value == -1 || neigh_g_value > s_g_value + 1)
          {
            uint x_f_value = s_g_value + 1 + (*h_func)(x_index);
            tree.add_to_open(x_index, s_g_value + 1, x_f_value, s_index);
          }
        }
      }
    }
  }

  ROS_DEBUG_STREAM("Reached vertices:");
  for (int i = 0; i < reachability.size(); i++)
  {
    std::vector<std::string> vs;
    for (const auto &vec : reachability[i])
    {
      std::string temp_str;
      for (bool v : vec)
      {
        temp_str += (v ? "Y" : "N");
      }
      vs.push_back(temp_str);
    }
    ROS_DEBUG_STREAM("vertex " << i << "\t" << vs);
  }

  ROS_WARN_STREAM("Can't find solution!");
  uint64_t total_states = 1;
  for (int i = 0; i < robot_number; i++)
  {
    total_states *= graph.size();
  }
  for (uint x : waypoints_number)
  {
    total_states *= (x + 1);
  }
  ROS_DEBUG_STREAM("Closed states: " << closed_count << "\tTotal states: " << total_states);

  return;
}

template <class T = uint(uint64_t)>
void search_function(const std::vector<std::vector<uint>> &waypoints, const mapd::mapd_state &is,
                     const std::vector<std::vector<uint>> &graph, const std::vector<uint> &robot_ids, T *h_func,
                     const std::vector<std::vector<uint>> &other_paths,
                     std::vector<std::vector<uint>> *result_task_paths,
                     std::vector<std::vector<uint>> *result_home_paths, const std::vector<bool> &going_home)
{
  ROS_DEBUG_STREAM("waypoints: " << std::endl << waypoints);
  ROS_DEBUG_STREAM("robot_ids: " << std::endl << robot_ids);
  ROS_DEBUG_STREAM("other_paths: " << std::endl << other_paths);
  int robot_number = robot_ids.size();
  std::vector<uint> waypoints_number;
  for (const auto &x : waypoints)
  {
    waypoints_number.push_back(x.size());
  }
  ROS_DEBUG_STREAM("instantiate tree");
  mapd::mapd_search_tree tree(graph, waypoints_number, robot_ids);
  int vertices_number = graph.size();
  uint64_t is_index = is.get_index_notation(vertices_number, waypoints_number);
  ROS_DEBUG_STREAM("initial state index notation: " << is_index);
  // ROS_DEBUG_STREAM("heuristic function: " << *h_func);
  // std::ofstream test("test_heuristic.txt");
  // test << *h_func;
  // test.close();
  uint h_value = (*h_func)(is_index);
  ROS_DEBUG_STREAM("initial state h-value: " << h_value);
  ROS_DEBUG_STREAM("add initial state to tree");
  tree.add_to_open(is_index, 0, h_value);
  ROS_DEBUG_STREAM("initial state: " << std::endl << is);
  ROS_DEBUG_STREAM("Starting state exploration...");
  uint64_t count = 0, closed_count = 0;
  auto start = std::chrono::system_clock::now();
  while (!tree.is_open_empty())
  {
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    if (elapsed.count() >= 10.0)
    {
      ROS_DEBUG_STREAM("Visited states: " << count);
      ROS_DEBUG_STREAM("Open queue size: " << tree.open_size());
      start = std::chrono::system_clock::now();
    }
    count++;
    uint64_t s_index = tree.get_next_state();
    // ROS_DEBUG_STREAM("VISIT: " << s_index);
    mapd::mapd_state s(s_index, vertices_number, waypoints_number, robot_ids);
    tree.set_state_to_visited(s_index);

    // check if this is a final state
    bool is_final = true;
    for (int i = 0; i < robot_number; i++)
    {
      if (s.waypoint_indices[i] != waypoints_number[i] - 1 || s.configuration[i] != waypoints[i].back())
      {
        is_final = false;
        break;
      }
    }

    if (is_final)
    {
      ROS_DEBUG_STREAM("found solution!");
      // reconstruct path
      std::vector<std::vector<uint>> task_path(robot_number), home_path(robot_number);
      try
      {
        while (true)
        {
          // ROS_DEBUG_STREAM("final path: " << s.configuration);
          for (int i = 0; i < robot_number; i++)
          {
            // the last condition is to make sure that the second last waypoint is inserted in the task path, not the
            // home path
            if (going_home[i] && s.waypoint_indices[i] == waypoints_number[i] - 1 &&
                s.configuration[i] != waypoints[i][s.waypoint_indices[i] - 1])
            {
              home_path[i].emplace(home_path[i].begin(), s.configuration[i]);
            }
            else
            {
              task_path[i].emplace(task_path[i].begin(), s.configuration[i]);
            }
          }
          s = mapd::mapd_state(tree.get_prev_state(s_index), vertices_number, waypoints_number, robot_ids);
          s_index = s.get_index_notation(vertices_number, waypoints_number);
        }
      }
      catch (const std::string &e)
      {
        // reached initial state
        ROS_DEBUG_STREAM("initial h-value: " << h_value);
        ROS_DEBUG_STREAM("path length: " << task_path[0].size() + home_path[0].size());
        ROS_DEBUG_STREAM("returning complete paths!");
        *result_task_paths = task_path;
        *result_home_paths = home_path;
        return;
      }
    }

    // search best state between neighbours and add to queue
    bool new_state_found = false;
    uint64_t best_state_index;
    uint best_state_f_value = std::numeric_limits<uint>::max();
    uint best_state_g_value;
    uint s_value = tree.visited_state_g(s_index);
    auto neigh_list = s.get_neigh(graph, waypoints);
    // if (neigh_list.empty())
    // {
    //   ROS_DEBUG_STREAM("Warning! Neighbour list is empty");
    // }
    // ROS_DEBUG_STREAM("Neighbours #: " << neigh_list.size());
    for (const mapd::mapd_state &x : neigh_list)
    {
      uint64_t x_index = x.get_index_notation(vertices_number, waypoints_number);
      if (x_index != s_index)
      {
        if (is_transition_valid(s, x, other_paths, tree.visited_state_g(s_index)))
        {
          // check that new state is not closed
          if (!tree.is_state_closed(x_index))
          {
            // check that new state is not visited
            if (!tree.is_state_visited(x_index))
            {
              // check that new state is not already in queue (unless found with a better path)
              if (!tree.is_state_in_queue(x_index) || tree.visited_state_g(x_index) > s_value + 1)
              {
                uint x_value = s_value + 1 + (*h_func)(x_index);
                if (x_value < best_state_f_value)
                {
                  best_state_f_value = x_value;
                  best_state_g_value = s_value + 1;
                  best_state_index = x_index;
                  new_state_found = true;
                }
              }
            }
          }
        }
      }
    }

    // best state is first inside map
    if (new_state_found)
    {
      tree.add_to_open(best_state_index, best_state_g_value, best_state_f_value, s_index);
    }
    else
    {
      // if a new state could not been found, it means that the current one is closed and can be removed from open queue
      tree.pop_next_state();
      tree.set_state_to_closed(s_index);
      closed_count++;
    }
  }

  ROS_WARN_STREAM("Can't find solution!");
  uint64_t total_states = 1;
  for (int i = 0; i < robot_number; i++)
  {
    total_states *= graph.size();
  }
  for (uint x : waypoints_number)
  {
    total_states *= x;
  }
  ROS_DEBUG_STREAM("Closed states: " << closed_count << "\tTotal states: " << total_states);
  return;
}

/*!
  Initializes the edge removal for the tests
*/
void OnlineDCOPTaskPlanner::init(int argc, char **argv)
{
  OnlineTaskPlanner::init(argc, argv);
  std::string str_time = current_time_str();
  log_ss << str_time << "\n"
         << "\tmapname: " << mapname << "\n"
         << "\ttask_set_file: " << task_set_file << "\n";
  // set edge list for removal tests
  if (mapname == "icelab_black")
  {
    edge_list = { { 28, 31, 20, edge_modification_plan::REMOVAL },
                  // { 28, 31, 15, edge_modification_plan::ADDITION },
                  { 45, 49, 40, edge_modification_plan::REMOVAL },
                  { 0, 1, 50, edge_modification_plan::REMOVAL },
                  { 42, 46, 60, edge_modification_plan::REMOVAL },
                  { 30, 31, 80, edge_modification_plan::REMOVAL } };
  }
  else if (mapname == "grid")
  {
    if (TEAM_SIZE == 6 || TEAM_SIZE == 2)
    {
      edge_list = { { 11, 16, 5, edge_modification_plan::REMOVAL },
                    { 13, 18, 10, edge_modification_plan::REMOVAL },
                    { 7, 12, 15, edge_modification_plan::REMOVAL },
                    { 7, 8, 20, edge_modification_plan::REMOVAL },
                    { 2, 7, 23, edge_modification_plan::REMOVAL},
                    { 1, 2, 23, edge_modification_plan::REMOVAL},
                    { 2, 3, 23, edge_modification_plan::REMOVAL},
                    { 15, 16, 25, edge_modification_plan::REMOVAL },
                    { 13, 14, 30, edge_modification_plan::REMOVAL } };
    }
    else if (TEAM_SIZE == 4)
    {
      edge_list = { { 11, 16, 5, edge_modification_plan::REMOVAL },
                    { 13, 18, 10, edge_modification_plan::REMOVAL },
                    { 7, 12, 15, edge_modification_plan::REMOVAL },
                    { 7, 8, 20, edge_modification_plan::REMOVAL },
                    { 1, 6, 23, edge_modification_plan::REMOVAL},
                    { 1, 2, 23, edge_modification_plan::REMOVAL},
                    { 0, 1, 23, edge_modification_plan::REMOVAL},
                    { 15, 16, 25, edge_modification_plan::REMOVAL },
                    { 13, 14, 30, edge_modification_plan::REMOVAL } };
    }
    else
    {
      edge_list = {};
    }
    
    
  }
  else
  {
    edge_list = {};
  }
}

/*!
  Callback for handling the token
*/
void OnlineDCOPTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{
  if (msg->ID_RECEIVER != TASK_PLANNER_ID)
    return;

  logistic_sim::Token token;
  token = *msg;
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;

  edges_mutex.lock();
  if (msg->INIT)
  {
    edges_mutex.unlock();
    init_token(msg, token);
  }
  else if (removed_edges.size() != 0 || added_edges.size() != 0)
  {
    // insert edges modifications inside token to notify the agents
    for (const logistic_sim::Edge &e : removed_edges)
    {
      token.REMOVED_EDGES.push_back(e);
    }
    removed_edges.clear();

    for (const logistic_sim::Edge &e : added_edges)
    {
      token.ADDED_EDGES.push_back(e);
    }
    added_edges.clear();
    edges_mutex.unlock();

    bool property_validity = check_conflict_free_property();
    ROS_ERROR_STREAM_COND(!property_validity, "Conflict-free property does not hold anymore!");
    ROS_INFO_STREAM_COND(property_validity, "Conflict-free property still holds true");
    if (!property_validity)
    {
      map_graph = build_graph();
      build_fw_matrix();
      // find a new home/recovery configuration configuration and insert inside the token
      std::vector<std::vector<uint>> waypoints;
      std::vector<uint> robot_ids;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        waypoints.push_back(dst_vertex);
        waypoints[i].push_back(src_vertex);
        robot_ids.push_back(i);
      }
      std::vector<uint> recovery_configuration = local_search_recovery_config(waypoints, robot_ids, {});
      if (recovery_configuration.empty())
      {
        ROS_ERROR_STREAM("Can't find a new home configuration, shutting down...");
        token.FAILED_REPAIR++;
        token.SHUTDOWN = true;
      }
      else
      {
        ROS_DEBUG_STREAM("new recovery configuration: " << recovery_configuration);
        for (int i = 0; i < recovery_configuration.size(); i++)
        {
          uint v = recovery_configuration[i];
          token.NEW_HOMES[i] = v;
          home_vertex[i] = v;
        }
      }
    }

    token.NEED_REPAIR = true;
    token.HAS_REPAIRED_PATH = std::vector<uint8_t>(TEAM_SIZE, false);
  }
  else if (msg->REPAIR)
  {
    edges_mutex.unlock();
    ROS_DEBUG_STREAM("Waiting for agents to repair their paths...");
  }
  else if (msg->MULTI_PLAN_REPAIR && !msg->SHUTDOWN)
  {
    edges_mutex.unlock();
    ROS_WARN_STREAM("For now we use centralized implementation of multi-robot repair!");
    // at this point, we have the robots' waypoints and their current position inside the token
    // run MAPD algorithm to find a solution and insert such solution inside the token

    multi_agent_repair(msg, token);
  }
  else if (msg->SINGLE_PLAN_REPAIR)
  {
    edges_mutex.unlock();
  }
  else
  {
    edges_mutex.unlock();

    mapf_attempts = 1;
    total_search_duration = 0.0;

    // ROS_INFO_STREAM("TODO DCOP TOKEN");
    token.HEADER.seq += 1;

    // the mutex is necessary because the window is updated in another thread
    window_mutex.lock();

    // the task planner has received the token after the allocation phase, hence it's completed
    if (allocation_phase)
    {
      allocation_phase = false;
      end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;
      times_file << "allocation:\t" << elapsed_seconds.count() << "\n";
      times_file.flush();

      if (offline_mode)
      {
        ROS_DEBUG("offline mode -- start time setted after allocation");
        start_time = ros::Time::now();
      }
    }

    // signals to the robots that new missions are in the token
    // robots will set this flag to false when this missions have been accepted
    if (!mission_windows.empty() && !token.NEW_MISSIONS_AVAILABLE)
    {
      if (!first_missions_sent)
      {
        first_missions_sent = true;
        first_valid_timestep = *std::max_element(msg->GOAL_STATUS.begin(), msg->GOAL_STATUS.end());
        ROS_DEBUG_STREAM("Starting timestep: " << first_valid_timestep);
      }

      start = std::chrono::system_clock::now();
      allocation_phase = true;
      token.MISSION = mission_windows.front();
      token.TAKEN_MISSION = std::vector<int>(token.MISSION.size(), -1);
      mission_windows.pop_front();
      // signals in the token when all the missions have been inserted
      if (missions.empty())
      {
        token.ALL_MISSIONS_INSERTED = true;
      }
      c_print("Mission window inserted in token - remaining windows: ", mission_windows.size(), yellow, P);
      token.NEW_MISSIONS_AVAILABLE = true;

      // mando il token al robot più scarico
      // std::vector<logistic_sim::Path> robot_paths(TEAM_SIZE);
      // for (int i=0; i<TEAM_SIZE; i++)
      // {
      //     robot_paths[i] = token.TRAILS[i];
      //     robot_paths[i].PATH.insert(robot_paths[i].PATH.end(), token.HOME_TRAILS[i].PATH.begin(),
      //     token.HOME_TRAILS[i].PATH.end());
      // }

      // int id_next_robot = 0;
      // int min_length = robot_paths[0].PATH.size();
      // for (int i=1; i<TEAM_SIZE; i++)
      // {
      //     if (robot_paths[i].PATH.size() < min_length)
      //     {
      //         min_length = robot_paths[i].PATH.size();
      //         id_next_robot = i;
      //     }
      // }

      // token.ID_RECEIVER = id_next_robot;
    }
    window_mutex.unlock();

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
      // simulate edge removal at fixed time steps
      if (EDGE_REMOVAL_TEST)
      {
        logistic_sim::ChangeEdgeRequest req;
        logistic_sim::ChangeEdgeResponse res;
        uint current_goal_status = *std::max_element(msg->GOAL_STATUS.begin(), msg->GOAL_STATUS.end());
        uint timestep = current_goal_status - first_valid_timestep;
        // auto current_time = std::chrono::system_clock::now();
        ros::Time current_time = ros::Time::now();
        ros::Duration diff = current_time - last_edge_removal;
        // std::chrono::duration<double> diff = current_time - last_edge_removal;
        if (first_valid_timestep != 0 && diff.toSec() >= 10.0 && !edge_list.empty())
        {
          for (auto it = edge_list.begin(); it != edge_list.end(); )
          {
            const edge_modification_plan &edge = *it;
            if (timestep >= edge.timestep)
            {
              if (edge.type == edge.REMOVAL)
              {
                ROS_INFO_STREAM("Removing edge (" << edge.from << "," << edge.to << ") at timestep " << timestep);
                req.remove = true;
                req.add = false;
                req.cost = 0;
              }
              else if (edge.type == edge.ADDITION)
              {
                ROS_INFO_STREAM("Adding edge (" << edge.from << "," << edge.to << ") at timestep " << timestep);
                req.remove = false;
                req.add = true;
                req.cost = 1;
              }
              req.u = edge.from;
              req.v = edge.to;
              change_edge(req, res);
              last_edge_removal = current_time;
              // edge_list.pop_front();
              it = edge_list.erase(it);
            }
            else
            {
              it++;
            }
          }
        }
      }
      // if (!first_missions_sent)
      // {
      //   first_valid_timestep = msg->GOAL_STATUS[0];
      //   ROS_DEBUG_STREAM("Starting timestep: " << first_valid_timestep);
      // }

      std::vector<logistic_sim::Path> paths = token.TRAILS;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        paths[i].PATH.insert(paths[i].PATH.end(), token.HOME_TRAILS[i].PATH.begin(), token.HOME_TRAILS[i].PATH.end());
      }
      check_paths_conflicts(paths);
      // check_paths_conflicts(token.TRAILS); VECCHIA VERSIONE
    }

    // updating robot stats from token
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      robots_data[i].interference_num = token.INTERFERENCE_COUNTER[i];
      robots_data[i].completed_missions = token.MISSIONS_COMPLETED[i];
      robots_data[i].completed_tasks = token.TASKS_COMPLETED[i];
      robots_data[i].tot_distance = token.TOTAL_DISTANCE[i];
      robots_data[i].total_time = (ros::Time::now() - start_time).sec;
    }

    // at shutdown stats must be written on disk
    if (token.SHUTDOWN)
    {
      times_file.close();
      boost::filesystem::path results_directory("results");
      if (!boost::filesystem::exists(results_directory))
      {
        boost::filesystem::create_directory(results_directory);
      }

      std::stringstream conf_dir_name;
      conf_dir_name << "results/" << name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << TEAM_SIZE
                    << "capacity" << CAPACITY[0] << "_" << mapname;
      boost::filesystem::path conf_directory(conf_dir_name.str());
      if (!boost::filesystem::exists(conf_directory))
      {
        boost::filesystem::create_directory(conf_directory);
      }

      int run_number = 1;
      std::stringstream filename;
      std::ifstream check_new;
      if (GENERATION != "file")
      {
        // checking file existence by name
        do
        {
          filename.str("");
          filename << conf_dir_name.str() << "/" << run_number << ".csv";
          check_new = std::ifstream(filename.str());
          run_number++;
        } while (check_new);
        check_new.close();
      }
      else
      {
        filename << conf_dir_name.str() << "/" << task_set_file << ".csv";
      }

      ofstream stats(filename.str());
      // the stream operator is defined in the taskplanner namespace
      taskplanner::operator<<(stats, robots_data);
      // stats << robots_data;
      stats.close();

      // write repair data
      std::stringstream repair_filename;
      repair_filename << filename.str() << "_repair_data.txt";
      ofstream repair_stats_file(repair_filename.str());
      repair_stats_file << "OBSTACLE_EVENTS: " << msg->OBSTACLE_EVENTS << "\n";
      repair_stats_file << "SUCCESSFULL_SA_REPAIR: " << msg->SUCCESSFULL_SA_REPAIR << "\n";
      repair_stats_file << "SUCCESSFULL_MA_REPAIR: " << msg->SUCCESSFULL_MA_REPAIR << "\n";
      repair_stats_file << "FAILED_REPAIR: " << msg->FAILED_REPAIR << "\n";
      repair_stats_file << "A* DURATIONS:";
      for (double d : astar_durations)
      {
        repair_stats_file << " " << d;
      }
      repair_stats_file << "\n# ATTEMPTS: ";
      for (uint a : astar_attempts)
      {
        repair_stats_file << " " << a;
      }

      repair_stats_file << "\nA* TIMESTEPS: ";
      for (uint a : astar_timesteps)
      {
        repair_stats_file << " " << a;
      }
      // repair_stats_file << "REPAIRS_PER_ROBOT:\n";
      // for (int i = 0; i < TEAM_SIZE; i++)
      // {
      //   repair_stats_file << msg->REPAIRS_PER_ROBOT[i] << " ";
      // }
      // repair_stats_file << "\n";
      repair_stats_file.close();

      for (int i = 0; i < TEAM_SIZE; i++)
      {
        std::stringstream ss;
        ss << filename.str() << "_robot_" << i << "_error.txt";
        ofstream amcl_error(ss.str());
        for (auto it = robot_pos_errors[i].begin(); it != robot_pos_errors[i].end(); it++)
        {
          amcl_error << *it << "\n";
        }
        amcl_error.close();
      }

      ros::NodeHandle nh;
      nh.setParam("/simulation_running", "false");
      ros::shutdown();
      // int cmd_result = system("./stop_experiment.sh");
    }
  }

  pub_token.publish(token);
  ros::spinOnce();
}

/*!
  Initializes the token and sets timers to detect stuck robots
*/
void OnlineDCOPTaskPlanner::init_token(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
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

/*!
  Implementation of the multi agent replanning
*/
void OnlineDCOPTaskPlanner::multi_agent_repair(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  // get valid paths
  std::vector<std::vector<uint>> other_paths;
  std::map<uint, std::vector<uint>> other_paths_map;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (msg->HAS_REPAIRED_PATH[i])
    {
      std::vector<uint> robot_path = msg->TRAILS[i].PATH;
      robot_path.insert(robot_path.end(), msg->HOME_TRAILS[i].PATH.begin(), msg->HOME_TRAILS[i].PATH.end());
      other_paths.push_back(robot_path);
      other_paths_map[i] = robot_path;
    }
  }

  // build map graph
  map_graph = build_graph();
  build_fw_matrix();
  std::vector<std::vector<uint>> waypoints;
  std::vector<uint> robot_ids;
  std::vector<bool> going_home;
  mapd::mapd_state initial_state;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (!msg->HAS_REPAIRED_PATH[i])
    {
      std::vector<uint> w = msg->ROBOT_WAYPOINTS[i].VERTICES;
      w.erase(w.begin());  // mapd planning algorithm does not need initial vertex in waypoints
      waypoints.push_back(w);
      // waypoints.push_back(msg->ROBOT_WAYPOINTS[i].VERTICES);
      robot_ids.push_back(i);
      going_home.push_back(true);

      initial_state.configuration.push_back(msg->TRAILS[i].PATH.front());
    }
  }
  initial_state.waypoint_indices = std::vector<uint>(robot_ids.size(), 0);
  initial_state.robot_ids = robot_ids;
  mapd::max_cost_heuristic h_func(map_graph, waypoints, robot_ids);
  // start search
  std::vector<std::vector<uint>> paths, home_paths;

  std::string str_time = current_time_str();
  log_ss << str_time << "\t"
         << "robot_ids:\n"
         << robot_ids << "\n";
  ROS_DEBUG_STREAM("robot_ids:\n" << robot_ids);
  log_ss << str_time << "\t"
         << "initial configuration:\n"
         << initial_state.configuration << "\n";
  ROS_DEBUG_STREAM("initial configuration:\n" << initial_state.configuration);
  log_ss << str_time << "\t"
         << "waypoints:\n"
         << waypoints << "\n";
  ROS_DEBUG_STREAM("waypoints:\n" << waypoints);
  log_ss << str_time << "\t"
         << "other paths:\n"
         << other_paths << "\n";
  ROS_DEBUG_STREAM("other paths:\n" << other_paths);

  mapd_time::state test_is;
  test_is.configuration = initial_state.configuration;
  test_is.waypoint_indices = initial_state.waypoint_indices;
  // test recovery configuration
  mapd_time::search_tree recovery_search_tree(map_graph, waypoints, test_is);
  // ROS_DEBUG_STREAM("searching recovery configuration...");
  // log_ss << str_time << "\t"
  //        << "searching recovery configuration..."
  //        << "\n";

  std::chrono::duration<double> search_duration;
  std::vector<std::vector<uint>> test_waypoints;
  std::vector<uint> test_robot_ids;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    test_robot_ids.push_back(i);
    test_waypoints.push_back(dst_vertex);
    test_waypoints[i].push_back(src_vertex);
  }

  // LOCAL SEARCH TEST
  // std::vector<uint> test_local_search =
  //     local_search_recovery_config(test_waypoints, test_robot_ids, {}, &search_duration);
  // ROS_DEBUG_STREAM("test local search configuration:\n"
  //                  << test_local_search << "time required: " << search_duration.count());

  // std::vector<uint> destination = find_best_recovery_config(waypoints, robot_ids, initial_state.configuration,
  // other_paths);
  // std::vector<uint> destination = recovery_search_tree.find_recovery_configuration(other_paths, robot_ids,
  // bind(&OnlineDCOPTaskPlanner::check_valid_recovery_configuration, this, _1, _2, _3));
  std::vector<uint> destination(robot_ids.size(), 0);
  for (int i = 0; i < robot_ids.size(); i++)
  {
    destination[i] = home_vertex[robot_ids[i]];
  }
  std::vector<std::vector<uint>> recovery_waypoints;
  for (int i = 0; i < destination.size(); i++)
  {
    recovery_waypoints.push_back({ destination[i] });
  }
  ROS_DEBUG_STREAM("recovery configuration:\n" << recovery_waypoints);
  log_ss << str_time << "\t"
         << "recovery configuration:\n"
         << recovery_waypoints << "\n";
  // test new state definition with no time included
  mapf::state test_mapf_is;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    test_mapf_is.configuration.push_back(msg->TRAILS[i].PATH.front());
  }
  std::vector<unsigned int> test_destination(TEAM_SIZE, 0);
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (!msg->HAS_REPAIRED_PATH[i])
    {
      test_destination[i] = home_vertex[i];
    }
  }

  // before starting MAPF search write current stats in file
  logistic_sim::Token temp_token = token;
  temp_token.OBSTACLE_EVENTS++;
  mapf_attempts--;
  if (mapf_attempts == 0)
  {
    write_statistics(temp_token);
  }
  else
  {
    write_statistics(temp_token, true);
  }
  mapf_attempts++;

  // mapf_attempts = 1;

  // First attempt to find a path to home configuration
  ROS_INFO_STREAM("Attempt " << mapf_attempts << "/" << max_mapf_attempts << " to find a path to home configuration");
  mapf::search_tree test_mapf(map_graph, test_destination, test_mapf_is, other_paths_map);
  uint time_limit = 1 * 30;
  double duration;
  std::list<mapf::state> result = test_mapf.astar_search(paths, time_limit, &duration);
  total_search_duration += duration;
  uint current_goal_status = *std::max_element(msg->GOAL_STATUS.begin(), msg->GOAL_STATUS.end());
  last_mapf_timestep = current_goal_status - first_valid_timestep;
  // reach recovery configuration with pseudo-dcop search
  // mapd_time::search_tree recovery_test_st(map_graph, recovery_waypoints, test_is);
  // recovery_test_st.dcop_search(paths, home_paths, other_paths, false);

  // test with time
  // mapd_time::search_tree test_st(map_graph, waypoints, test_is);
  // test_st.dcop_search(paths, home_paths, other_paths);

  // test with no other paths
  // astar_search_function(waypoints, initial_state, map_graph, robot_ids, &h_func, std::vector<std::vector<uint>>(),
  //                       &paths, &home_paths, going_home);
  // search_function(waypoints, initial_state, map_graph, robot_ids, &h_func, other_paths, &paths, &home_paths,
  // going_home);

  // update last goal time to prevent shutdown
  last_goal_time = ros::Time::now();
  // if result is not empty then MA search has found valid paths
  // Paths are inserted in the token,
  // MA stats are updated
  // and token state flags are updated
  if (!result.empty())
  {
    astar_durations.push_back(total_search_duration);
    astar_attempts.push_back(mapf_attempts);
    
    astar_timesteps.push_back(last_mapf_timestep);
    previous_attempts_configs.clear();
    mapf_attempts = 1;
    // insert paths inside token
    for (int i = 0; i < paths.size(); i++)
    {
      uint robot_id = robot_ids[i];
      // token.TRAILS[robot_id].PATH = paths[i];
      token.HOME_TRAILS[robot_id].PATH = paths[i];
      // token.TRAILS[robot_id].PATH.insert(token.TRAILS[robot_id].PATH.end(), paths[i].begin(), paths[i].end());
      // token.HOME_TRAILS[robot_id].PATH = home_paths[i];
      ROS_DEBUG_STREAM("final task path: " << paths[i]);
      log_ss << str_time << "\t"
             << "final task path: " << paths[i] << "\n";
      // ROS_DEBUG_STREAM("final home path: " << home_paths[i]);
    }

    // token.HAS_REPAIRED_PATH = std::vector<uint8_t>(TEAM_SIZE, true);
    token.MULTI_PLAN_REPAIR = false;
    token.SINGLE_PLAN_REPAIR = true;
    token.SUCCESSFULL_MA_REPAIR++;
    // write_statistics(token, true);
  }
  // if search has failed and the max number of attempts
  // has been reached, then the shutdown is issued
  else if (mapf_attempts == max_mapf_attempts)
  {
    std::string log_filename = current_time_filename_str() + ".log";
    ROS_ERROR_STREAM("Could not find a MA path to recovery configuration!!! - shutting down...");
    ROS_ERROR_STREAM("Logging MA data to " << log_filename);
    std::ofstream of(log_filename);
    of << log_ss.str();
    of.close();
    token.FAILED_REPAIR++;
    token.OBSTACLE_EVENTS++;
    write_statistics(token, true);
    token.SHUTDOWN = true;
    // getc(stdin);
  }
  // search has failed, start local search
  // to generate a new home configuration
  else
  {
    logistic_sim::Token temp_token = token;
    temp_token.OBSTACLE_EVENTS++;
    write_statistics(temp_token, true);
    mapf_attempts++;
    previous_attempts_configs.push_back(test_destination);
    std::vector<std::vector<uint>> waypoints;
    std::vector<uint> robot_ids;
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      waypoints.push_back(dst_vertex);
      waypoints[i].push_back(src_vertex);
      robot_ids.push_back(i);
    }
    std::vector<uint> recovery_configuration = local_search_recovery_config(waypoints, robot_ids, {});
    if (recovery_configuration.empty() && mapf_attempts == max_mapf_attempts)
    {
      ROS_ERROR_STREAM("Can't find a new home configuration, shutting down...");
      token.FAILED_REPAIR++;
      token.OBSTACLE_EVENTS++;
      token.SHUTDOWN = true;
    }
    else if (recovery_configuration.empty())
    {
      mapf_attempts++;
    }
    else
    {
      ROS_DEBUG_STREAM("new recovery configuration: " << recovery_configuration);
      for (int i = 0; i < recovery_configuration.size(); i++)
      {
        uint v = recovery_configuration[i];
        token.NEW_HOMES[i] = v;
        home_vertex[i] = v;
      }
      token.MULTI_PLAN_REPAIR = false;
      token.REPAIR = true;
      token.HAS_REPAIRED_PATH = std::vector<uint8_t>(TEAM_SIZE, false);
    }
  }
}

void OnlineDCOPTaskPlanner::advertise_change_edge_service(ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM("Advertising change_edge service");
  change_edge_service = nh.advertiseService("change_edge", &OnlineDCOPTaskPlanner::change_edge, this);
  if (!change_edge_service)
  {
    ROS_ERROR_STREAM("Can't create change_edge service");
  }
  else
  {
    ROS_INFO_STREAM("change_edge service advertised successfully");
  }
}

void OnlineDCOPTaskPlanner::print_graph()
{
  std::ofstream test("test-map.graph");
  for (int i = 0; i < dimension; i++)
  {
    test << vertex_web[i].id << std::endl;
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      test << "\t(" << vertex_web[i].id << "," << vertex_web[i].id_neigh[j] << ")" << std::endl;
    }
  }
  test.close();
}

// build alternative represntation for the graph
// useful for the MA replanner
std::vector<std::vector<unsigned int>> OnlineDCOPTaskPlanner::build_graph()
{
  std::vector<std::vector<unsigned int>> result = std::vector<std::vector<unsigned int>>(dimension);
  for (int i = 0; i < dimension; i++)
  {
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      result[i].push_back(vertex_web[i].id_neigh[j]);
    }
  }

  return result;
}

// callback for the /change_edge service
bool OnlineDCOPTaskPlanner::change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res)
{
  int result;

  if (msg.remove)
  {
    std::string time_str = current_time_str();
    ROS_INFO_STREAM("Requested remotion of edge (" << msg.u << "," << msg.v << ")");
    log_ss << time_str << "\t"
           << "Requested remotion of edge (" << msg.u << "," << msg.v << ")"
           << "\n";
    print_graph();
    result = RemoveEdge(vertex_web, dimension, msg.u, msg.v);
    ROS_ERROR_STREAM_COND(result > 0, "Failed remotion!");
    // ROS_ERROR_STREAM_COND(result == 1, "Can't find vertex " << msg.u);
    // ROS_ERROR_STREAM_COND(result == 4, "Can't find vertex " << msg.v);
    // ROS_ERROR_STREAM_COND(result == 2, "Can't find edge (" << msg.u << "," << msg.v << ")");
    // ROS_ERROR_STREAM_COND(result == 3, "Can't find edge (" << msg.v << "," << msg.u << ")");
    if (result == 0)
    {
      edges_mutex.lock();

      logistic_sim::Edge e;
      e.u = msg.u;
      e.v = msg.v;
      removed_edges.push_back(e);

      edges_mutex.unlock();

      res.result = true;
      return true;
    }
    else
    {
      res.result = false;
      return false;
    }
  }
  else if (msg.add)
  {
    ROS_INFO_STREAM("Requested addition of edge (" << msg.u << "," << msg.v << ")");
    result = AddEdge(vertex_web, dimension, msg.u, msg.v, msg.cost);
    ROS_ERROR_STREAM_COND(result > 0, "Failed addition!");

    if (result == 0)
    {
      edges_mutex.lock();

      logistic_sim::Edge e;
      e.u = msg.u;
      e.v = msg.v;
      added_edges.push_back(e);

      edges_mutex.unlock();

      res.result = true;
      return true;
    }
    else
    {
      res.result = false;
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Malformed request for ChangeEdge");
    result = false;
  }

  res.result = result;
  return result;
}

struct conflict_free_state
{
  uint g, f, vertex;
  int prev;

  conflict_free_state() : g(0), f(0), vertex(0), prev(-1)
  {
  }

  bool operator<(const conflict_free_state &other)
  {
    if (f < other.f)
      return true;
    else if (f == other.f && g > other.g)
      return true;
    return false;
  }
};

/*!
  implementaion of the conflict free check
  implements tree search but tells which robots can reach a particular vertex from home
  for now no heuristic
  use reverse search, search path from vertex to
*/
std::vector<bool> OnlineDCOPTaskPlanner::_check_conflict_free_impl(uint task_endpoint, std::vector<uint> *homes)
{
  std::vector<uint> destinations;
  if (homes != nullptr)
  {
    destinations = *homes;
  }
  else
  {
    destinations = home_vertex;
  }
  int found_count = 0;
  std::vector<bool> result(TEAM_SIZE, false);
  std::map<uint, conflict_free_state> visited;
  std::map<uint, conflict_free_state> open;

  conflict_free_state initial_state;
  initial_state.g = 0;
  initial_state.f = 0;

  open[task_endpoint] = initial_state;

  while (!open.empty())
  {
    auto u_it = open.begin();
    uint u_id = u_it->first;
    conflict_free_state u_state = u_it->second;
    open.erase(u_id);
    // ROS_DEBUG_STREAM(u_id);

    visited[u_id].f = u_state.f;
    visited[u_id].g = u_state.g;
    visited[u_id].prev = u_state.prev;

    // reached home vertex
    auto it = std::find(destinations.begin(), destinations.end(), u_id);
    if (it != destinations.end())
    {
      int pos = it - destinations.begin();
      // ROS_DEBUG_STREAM(pos);
      if (result[pos] == false)
      {
        result[pos] = true;
        found_count++;
        if (found_count == TEAM_SIZE)
          return result;
      }
    }
    else
    {
      // check neighbour states
      for (int i = 0; i < vertex_web[u_id].num_neigh; i++)
      {
        uint v_id = vertex_web[u_id].id_neigh[i];
        if (!visited.count(v_id) && !open.count(v_id))
        {
          conflict_free_state v_state;
          v_state.g = u_state.g + 1;
          v_state.f = v_state.g;
          v_state.prev = u_id;
          open[v_id] = v_state;
        }
      }
    }
  }

  return result;
}

/*!
  Checks if the configuration given in input is a good home configuration
  \param homes pointer to the tested configuration
  \param reachability_factor pointer to double, the function writes over it the percentage of reachable homes, used by the local search heuristic
  \param print tells whether to print debug information
*/
bool OnlineDCOPTaskPlanner::check_conflict_free_property(std::vector<uint> *homes, double *reachability_factor,
                                                         bool print)
{
  std::vector<uint> destinations;
  if (homes != nullptr)
  {
    destinations = *homes;
  }
  else
  {
    destinations = home_vertex;
  }
  uint reached_count = 0, total_count = 0;
  std::vector<bool> property_validity(TEAM_SIZE, true);
  std::vector<uint> task_endpoints = dst_vertex;
  task_endpoints.push_back(src_vertex);
  for (uint v : task_endpoints)
  {
    auto result = _check_conflict_free_impl(v, homes);
    if (print)
    {
      ROS_DEBUG_STREAM("Vertex " << v << ":\t");
    }
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      total_count++;
      bool r = result[i];
      if (print)
      {
        ROS_DEBUG_STREAM_COND(r, " Y to " << destinations[i]);
        ROS_DEBUG_STREAM_COND(!r, " N to " << destinations[i]);
      }
      if (r == false)
      {
        property_validity[i] = false;
      }
      else
      {
        reached_count++;
      }
    }
  }

  if (reachability_factor != nullptr)
  {
    *reachability_factor = static_cast<double>(reached_count) / static_cast<double>(total_count);
  }
  for (bool r : property_validity)
  {
    if (r == false)
    {
      return false;
    }
  }
  return true;
}

void OnlineDCOPTaskPlanner::write_statistics(const logistic_sim::Token &msg, bool consider_current_search)
{
  boost::filesystem::path results_directory("results");
  if (!boost::filesystem::exists(results_directory))
  {
    boost::filesystem::create_directory(results_directory);
  }

  std::stringstream conf_dir_name;
  conf_dir_name << "results/" << name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << TEAM_SIZE << "capacity"
                << CAPACITY[0] << "_" << mapname;
  boost::filesystem::path conf_directory(conf_dir_name.str());
  if (!boost::filesystem::exists(conf_directory))
  {
    boost::filesystem::create_directory(conf_directory);
  }

  int run_number = 1;
  std::stringstream filename;
  std::ifstream check_new;
  if (GENERATION != "file")
  {
    // checking file existence by name
    do
    {
      filename.str("");
      filename << conf_dir_name.str() << "/" << run_number << ".temp.csv";
      check_new = std::ifstream(filename.str());
      run_number++;
    } while (check_new);
    check_new.close();
  }
  else
  {
    filename << conf_dir_name.str() << "/" << task_set_file << ".temp.csv";
  }

  ofstream stats(filename.str());
  // the stream operator is defined in the taskplanner namespace
  taskplanner::operator<<(stats, robots_data);
  // stats << robots_data;
  stats.close();

  // write repair data
  std::stringstream repair_filename;
  repair_filename << filename.str() << "_temp_repair_data.txt";
  ofstream repair_stats_file(repair_filename.str());
  repair_stats_file << "OBSTACLE_EVENTS: " << msg.OBSTACLE_EVENTS << "\n";
  repair_stats_file << "SUCCESSFULL_SA_REPAIR: " << msg.SUCCESSFULL_SA_REPAIR << "\n";
  repair_stats_file << "SUCCESSFULL_MA_REPAIR: " << msg.SUCCESSFULL_MA_REPAIR << "\n";
  repair_stats_file << "FAILED_REPAIR: " << msg.FAILED_REPAIR << "\n";
  repair_stats_file << "A* DURATIONS:";
  for (double d : astar_durations)
  {
    repair_stats_file << " " << d;
  }
  if (consider_current_search)
  {
    repair_stats_file << " " << total_search_duration;
  }

  repair_stats_file << "\n# ATTEMPTS: ";
  for (uint a : astar_attempts)
  {
    repair_stats_file << " " << a;
  }
  if (consider_current_search)
  {
    repair_stats_file << " " << mapf_attempts;
  }

  repair_stats_file << "\nA* TIMESTEPS: ";
  for (uint a : astar_timesteps)
  {
    repair_stats_file << " " << a;
  }
  if (consider_current_search)
  {
    repair_stats_file << " " << last_mapf_timestep;
  }
  
  // repair_stats_file << "REPAIRS_PER_ROBOT:\n";
  // for (int i = 0; i < TEAM_SIZE; i++)
  // {
  //   repair_stats_file << msg.REPAIRS_PER_ROBOT[i] << " ";
  // }
  // repair_stats_file << "\n";
  repair_stats_file.close();

  for (int i = 0; i < TEAM_SIZE; i++)
  {
    std::stringstream ss;
    ss << filename.str() << "_robot_" << i << "_error.txt";
    ofstream amcl_error(ss.str());
    for (auto it = robot_pos_errors[i].begin(); it != robot_pos_errors[i].end(); it++)
    {
      amcl_error << *it << "\n";
    }
    amcl_error.close();
  }
}

bool reachability_test(uint from, const std::vector<uint> &destinations, const std::vector<uint> &banned_vertices,
                       vertex *vertex_web, uint *reached_destinations = nullptr)
{
  uint found_count = 0;
  std::map<uint, conflict_free_state> visited;
  // std::map<uint, conflict_free_state> open;
  std::set<conflict_free_state, std::function<bool(const conflict_free_state &, const conflict_free_state &)>> open(
      [&](const conflict_free_state &lhs, const conflict_free_state &rhs) {
        if (lhs.f < rhs.f)
        {
          return true;
        }
        else if (lhs.f == rhs.f)
        {
          if (lhs.g > rhs.g)
          {
            return true;
          }
          else if (lhs.g == rhs.g)
          {
            return lhs.vertex < rhs.vertex;
          }
        }
        return false;
      });

  conflict_free_state initial_state;
  initial_state.g = 0;
  initial_state.f = 0;
  initial_state.vertex = from;

  // open[from] = initial_state;
  open.insert(initial_state);

  if (reached_destinations != nullptr)
  {
    *reached_destinations = 0;
  }

  while (!open.empty())
  {
    auto u_it = open.begin();
    // uint u_id = u_it->first;
    // conflict_free_state u_state = u_it->second;
    conflict_free_state u_state = *u_it;
    uint u_id = u_state.vertex;
    // open.erase(u_id);
    open.erase(u_state);
    // ROS_DEBUG_STREAM(u_id);

    visited[u_id].f = u_state.f;
    visited[u_id].g = u_state.g;
    visited[u_id].prev = u_state.prev;

    // reached home vertex
    auto it = std::find(destinations.begin(), destinations.end(), u_id);
    auto banned_it = std::find(banned_vertices.begin(), banned_vertices.end(), u_id);
    if (it != destinations.end())
    {
      int pos = it - destinations.begin();
      found_count++;
      if (reached_destinations != nullptr)
      {
        *reached_destinations = found_count;
      }
      if (found_count == destinations.size())
        return true;
    }
    else if (banned_it == banned_vertices.end())
    {
      // check neighbour states
      for (int i = 0; i < vertex_web[u_id].num_neigh; i++)
      {
        uint v_id = vertex_web[u_id].id_neigh[i];
        bool v_in_open = false;
        for (const conflict_free_state &v : open)
        {
          if (v.vertex == v_id)
          {
            v_in_open = true;
            break;
          }
        }
        if (!visited.count(v_id) && !v_in_open /*!open.count(v_id)*/)
        {
          conflict_free_state v_state;
          v_state.g = u_state.g + 1;
          v_state.f = v_state.g;
          v_state.prev = u_id;
          v_state.vertex = v_id;
          // open[v_id] = v_state;
          open.insert(v_state);
        }
      }
    }
  }

  return false;
}

bool OnlineDCOPTaskPlanner::check_valid_recovery_configuration(const std::vector<uint> &configuration,
                                                               const std::vector<uint> &robot_ids,
                                                               const std::vector<std::vector<uint>> &waypoints,
                                                               double *reachability_factor,
                                                               std::vector<uint> *reached_per_robot,
                                                               std::vector<uint> *destinations_per_robot)
{
  uint total_destinations = 0;
  uint reached_destinations = 0;
  bool valid_configuration = true;
  for (int i = 0; i < waypoints[0].size(); i++)
  {
    if (destinations_per_robot != nullptr)
    {
      destinations_per_robot->push_back(configuration.size());
    }
    uint from = waypoints[0][i];
    std::vector<uint> banned_vertices = {};
    std::vector<uint> destinations = configuration;

    // add other homes to banned vertices and own home to destinations
    // for (int j = 0; j < home_vertex.size(); j++)
    // {
    //   if (home_vertex[j] == robot_ids[i])
    //   {
    //     destinations.push_back(home_vertex[j]);
    //   }
    //   else
    //   {
    //     banned_vertices.push_back(home_vertex[j]);
    //   }
    // }

    uint robot_reached_destinations;
    if (!reachability_test(from, destinations, banned_vertices, vertex_web, &robot_reached_destinations))
    {
      valid_configuration = false;
    }
    if (reached_per_robot != nullptr)
    {
      reached_per_robot->push_back(robot_reached_destinations);
    }
    total_destinations += destinations.size();
    reached_destinations += robot_reached_destinations;
  }
  if (reachability_factor != nullptr)
  {
    *reachability_factor = static_cast<double>(reached_destinations) / static_cast<double>(total_destinations);
  }
  return valid_configuration;
}

void enumerate_configs(std::vector<std::vector<uint>> &result, std::vector<uint> &temp_config, uint num_vertices,
                       uint robot_number, uint iteration = 0)
{
  for (uint i = 0; i < num_vertices; i++)
  {
    temp_config[iteration] = i;
    if (iteration < robot_number - 1)
    {
      enumerate_configs(result, temp_config, num_vertices, robot_number, iteration + 1);
    }
    else
    {
      result.push_back(temp_config);
    }
  }
}

std::vector<std::vector<uint>> OnlineDCOPTaskPlanner::find_all_recovery_configs(
    const std::vector<std::vector<uint>> &waypoints, const std::vector<uint> &robot_ids)
{
  uint robot_number = waypoints.size();
  std::vector<std::vector<uint>> configurations, result;
  std::vector<uint> temp_config(robot_number, 0);
  enumerate_configs(configurations, temp_config, map_graph.size(), robot_number);

  for (const std::vector<uint> &conf : configurations)
  {
    if (check_valid_recovery_configuration(conf, robot_ids, waypoints))
    {
      result.push_back(conf);
    }
  }
  return result;
}

uint max_cost_heuristic(const std::vector<uint> &current_config, const std::vector<uint> &final_config,
                        const std::vector<std::vector<uint>> &fw_matrix)
{
  uint robot_number = current_config.size();
  uint result = 0;
  for (uint i = 0; i < robot_number; i++)
  {
    if (fw_matrix[current_config[i]][final_config[i]] > result)
    {
      result = fw_matrix[current_config[i]][final_config[i]];
    }
  }
  return result;
}

std::vector<uint> OnlineDCOPTaskPlanner::find_best_recovery_config(const std::vector<std::vector<uint>> &waypoints,
                                                                   const std::vector<uint> &robot_ids,
                                                                   const std::vector<uint> &current_config,
                                                                   const std::vector<std::vector<uint>> &other_paths)
{
  uint robot_number = waypoints.size();
  std::vector<std::vector<uint>> configurations;
  auto valid_configs =
      std::set<std::vector<uint>, std::function<bool(const std::vector<uint> &, const std::vector<uint> &)>>{ [&](
          const std::vector<uint> &lhs, const std::vector<uint> &rhs) {
        // max cost heuristic
        if (max_cost_heuristic(current_config, lhs, fw_matrix) < max_cost_heuristic(current_config, rhs, fw_matrix))
        {
          return true;
        }
        else
        {
          return false;
        }
      } };
  std::vector<uint> temp_config(robot_number, 0);
  enumerate_configs(configurations, temp_config, map_graph.size(), robot_number);

  for (const std::vector<uint> &conf : configurations)
  {
    if (check_valid_recovery_configuration(conf, robot_ids, waypoints))
    {
      // check if configuration can cause trouble to other robots
      uint h_val = max_cost_heuristic(current_config, conf, fw_matrix);
      bool good = true;
      for (int i = 0; i < other_paths.size(); i++)
      {
        for (uint v : conf)
        {
          if (h_val < other_paths[i].size() &&
              std::find(other_paths[i].begin() + h_val, other_paths[i].end(), v) != other_paths[i].end())
          {
            good = false;
          }
        }
      }
      if (good)
      {
        valid_configs.emplace(conf);
      }
    }
  }
  if (valid_configs.empty())
  {
    ROS_ERROR_STREAM("Can't find a valid recovery configuration!");
    return std::vector<uint>(robot_number, 0);
  }
  else
  {
    return *valid_configs.begin();
  }
}

void OnlineDCOPTaskPlanner::_generate_near_configs_impl(std::vector<std::vector<uint>> &result,
                                                        const std::vector<uint> &s, std::vector<uint> &temp_state,
                                                        unsigned int robot_i)
{
  unsigned int current_vertex = s[robot_i], robots_number = s.size();
  std::vector<unsigned int> near_vertices;
  near_vertices = { current_vertex };
  for (uint v : map_graph[current_vertex])
  {
    if (v != src_vertex && std::find(dst_vertex.begin(), dst_vertex.end(), v) == dst_vertex.end())
    {
      near_vertices.push_back(v);
    }
  }

  for (unsigned int v : near_vertices)
  {
    temp_state[robot_i] = v;

    if (robot_i < robots_number - 1)
    {
      _generate_near_configs_impl(result, s, temp_state, robot_i + 1);
    }
    else
    {
      // check conflict state
      bool good = true;

      if (s == temp_state)
      {
        good = false;
      }

      for (int i = 0; i < robots_number; i++)
      {
        for (int j = i + 1; j < robots_number; j++)
        {
          if (temp_state[i] == temp_state[j])
          {
            good = false;
          }
        }
      }

      if (good)
      {
        result.push_back(temp_state);
      }
    }
  }
}

std::vector<std::vector<uint>> OnlineDCOPTaskPlanner::generate_near_configs(const std::vector<uint> &config)
{
  std::vector<std::vector<uint>> result;
  std::vector<uint> temp_state = config;
  _generate_near_configs_impl(result, config, temp_state, 0);
  return result;
}

std::vector<uint> OnlineDCOPTaskPlanner::create_random_config(
    const std::vector<uint> &valid_vertices, uint robot_number,
    const std::set<std::vector<uint>, std::function<bool(const std::vector<uint> &, const std::vector<uint> &)>>
        &visited_states)
{
  // static std::random_device rd;
  // static std::mt19937 gen(rd());
  // use fixed seed for testing purposes and reproducibility
  // constexpr unsigned int seed = 5;
  // static std::mt19937 gen(seed);
  bool found = false;
  std::vector<uint> random_config(robot_number);
  while (!found)
  {
    std::vector<uint> pool = valid_vertices;
    for (int i = 0; i < robot_number; i++)
    {
      std::uniform_int_distribution<> dis(0, pool.size() - 1);
      uint index = dis(gen);
      random_config[i] = pool[index];
      pool.erase(pool.begin() + index);
    }
    if (visited_states.count(random_config) == 0)
    {
      // restart_count++;
      found = true;
      // current_configuration = random_config;
    }
  }

  return random_config;
}

/*!
  Implements the local search used by the MA replanner
  Uses random restart hill-climbing to find a configuration
  where the check_conflict_free_property() holds true.
  Uses the number of homes reachable from the near configurations
  as heuristic to guid the search
  \param robot_ids vector with the ids of the agents that are replanning
  \param other_paths vector with the paths of the agents that are not replanning
  \param search_duration stores the duration of the local search
*/
std::vector<uint> OnlineDCOPTaskPlanner::local_search_recovery_config(const std::vector<std::vector<uint>> &waypoints,
                                                                      const std::vector<uint> &robot_ids,
                                                                      const std::vector<std::vector<uint>> &other_paths,
                                                                      std::chrono::duration<double> *search_duration)
{
  struct edge_count
  {
    uint vertex;
    uint edges;

    bool operator<(const edge_count &other) const
    {
      if (edges < other.edges)
      {
        return true;
      }
      else if (edges == other.edges)
      {
        return vertex < other.vertex;
      }
    }
  };
  uint robot_number = robot_ids.size();
  std::vector<uint> starting_configuration(robot_number);
  uint restart_count = 0, visited_count = 1;
  double max_factor = 0.0;
  std::vector<uint> max_factor_config, max_factor_reached_destinations, max_factor_total_destinations;

  std::set<edge_count> vertex_set;
  std::vector<uint> valid_vertices;
  for (int i = 0; i < map_graph.size(); i++)
  {
    if (std::find(dst_vertex.begin(), dst_vertex.end(), i) == dst_vertex.end() && src_vertex != i)
    {
      uint count = map_graph[i].size();
      edge_count ec;
      ec.vertex = i;
      ec.edges = count;
      vertex_set.insert(ec);
      valid_vertices.push_back(i);
    }
  }

  auto it = vertex_set.rbegin();
  for (int i = 0; i < robot_number; i++)
  {
    starting_configuration[i] = it->vertex;
    it++;
  }

  starting_configuration = create_random_config(valid_vertices, robot_number, {});

  // choose a random permutation of the starting configuration
  {
    std::vector<uint> temp_config = starting_configuration, new_config;
    for (uint i=0; i<robot_number; i++)
    {
      std::uniform_int_distribution<> dis(0, temp_config.size() - 1);
      uint index = dis(gen);
      auto it = temp_config.begin() + index;
      new_config.push_back(*it);
      temp_config.erase(it);
    }

    starting_configuration = new_config;
  }

  auto cmp_function = [&](const std::vector<uint> &lhs, const std::vector<uint> &rhs) {
    for (int i = 0; i < lhs.size(); i++)
    {
      if (lhs[i] < rhs[i])
      {
        return true;
      }
      else if (lhs[i] > rhs[i])
      {
        return false;
      }
    }
    return false;
  };
  std::set<std::vector<uint>, std::function<bool(const std::vector<uint> &, const std::vector<uint> &)>> visited_states(
      cmp_function),
      unvisited_states(cmp_function);


  std::vector<uint> current_configuration = starting_configuration;
  double time_limit = 1 * 60;
  auto start_time = std::chrono::system_clock::now();
  bool good = true;
  while (good)
  {
    auto current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> delta = current_time - start_time;
    if (delta.count() > time_limit)
    {
      if (search_duration != nullptr)
      {
        *search_duration = delta;
      }
      good = false;
    }

    // visited_count++;
    visited_states.insert(current_configuration);
    if (unvisited_states.find(current_configuration) != unvisited_states.end())
    {
      unvisited_states.erase(current_configuration);
    }

    double reachability_factor;
    // if (check_valid_recovery_configuration(current_configuration, robot_ids, waypoints, &reachability_factor))
    if (check_conflict_free_property(&current_configuration, &reachability_factor, false))
    {
      if (std::find(previous_attempts_configs.begin(), previous_attempts_configs.end(), current_configuration) ==
          previous_attempts_configs.end())
      {
        if (search_duration != nullptr)
        {
          *search_duration = delta;
        }
        ROS_DEBUG_STREAM("# of restarts: " << restart_count << "\tmax_factor: " << max_factor
                                           << "\t# of visited states: " << visited_count);
        return current_configuration;
      }
    }
    else
    {
      // check neighbours for better state
      std::vector<uint> best_config;
      double best_value = 0.0;
      std::vector<std::vector<uint>> near_configs = generate_near_configs(current_configuration);
      for (auto &c : near_configs)
      {
        double d;
        std::vector<uint> a, b;
        // if (check_valid_recovery_configuration(c, robot_ids, waypoints, &d, &a, &b))
        if (visited_states.find(c) == visited_states.end())
        {
          visited_count++;
          bool found = check_conflict_free_property(&c, &d, false);
          if (unvisited_states.find(c) == unvisited_states.end())
          {
            unvisited_states.insert(c);
          }
          if (d > reachability_factor && d > best_value)
          {
            if (d > max_factor)
            {
              max_factor = d;
              max_factor_config = c;
              // max_factor_reached_destinations = a;
              // max_factor_total_destinations = b;
            }
            reachability_factor = d;
            best_value = d;
            best_config = c;
          }

          if (found && std::find(previous_attempts_configs.begin(), previous_attempts_configs.end(), c) ==
              previous_attempts_configs.end())
          {
            if (search_duration != nullptr)
            {
              *search_duration = delta;
            }
            ROS_DEBUG_STREAM("# of restarts: " << restart_count << "\tmax_factor: " << max_factor
                                               << "\t# of visited states: " << visited_count);
            return c;
          }
        }
      }

      if (best_value > 0.0)
      {
        current_configuration = best_config;
      }
      else
      {
        current_configuration = create_random_config(valid_vertices, robot_number, visited_states);
        restart_count++;
      }
      // else if (!unvisited_states.empty())
      // {
      //   current_configuration = *unvisited_states.begin();
      // }
      // else
      // {
      //   if (search_duration != nullptr)
      //   {
      //     *search_duration = delta;
      //   }
      //   good = false;
      // }
    }
  }

  ROS_DEBUG_STREAM("# of restarts: " << restart_count << "\tmax_factor: " << max_factor << "\t# of visited states: "
                                     << visited_count << "\tmax_factor config: " << max_factor_config);
  //  << "\tmax_factor reached: " << max_factor_reached_destinations
  //  << "\tmax_factor total destinations " << max_factor_total_destinations);
  ROS_WARN_STREAM("Can't find new recovery configuration!");
  return std::vector<uint>();
}

void OnlineDCOPTaskPlanner::build_fw_matrix()
{
  uint vertex_number = map_graph.size();
  uint max_v = std::numeric_limits<uint>::max();
  fw_matrix = std::vector<std::vector<uint>>(vertex_number, std::vector<uint>(vertex_number, max_v));
  for (uint i = 0; i < vertex_number; i++)
  {
    fw_matrix[i][i] = 0;
    for (uint u : map_graph[i])
    {
      fw_matrix[i][u] = 1;
    }
  }

  for (uint k = 0; k < vertex_number; k++)
  {
    for (uint i = 0; i < vertex_number; i++)
    {
      for (uint j = 0; j < vertex_number; j++)
      {
        if (fw_matrix[i][k] != max_v && fw_matrix[k][j] != max_v && fw_matrix[i][j] > fw_matrix[i][k] + fw_matrix[k][j])
        {
          fw_matrix[i][j] = fw_matrix[i][k] + fw_matrix[k][j];
        }
      }
    }
  }
}

}  // namespace onlinedcoptaskplanner

void crash_handler(int sig)
{
  void *array[15];
  size_t size;

  size = backtrace(array, 15);

  fprintf(stderr, "printing error to file...\n");
  FILE *fp = fopen("crash_stacktrace.log", "w");
  fprintf(fp, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, fileno(fp));
  fclose(fp);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinedcoptaskplanner::OnlineDCOPTaskPlanner ODTP(nh_);
  ODTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // if (signal(SIGSEGV, crash_handler) == SIG_ERR)
  // {
  //   std::cerr << "can't print stack trace" << std::endl;
  // }
  ODTP.run();
  sleep(3);
  int cmd_result = system("tmux kill-session -t log_sim");
  return 0;
}
