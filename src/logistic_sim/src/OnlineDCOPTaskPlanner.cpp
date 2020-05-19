#include <execinfo.h>
#include <signal.h>
#include <chrono>
#include <unordered_map>
#include "OnlineDCOPTaskPlanner.hpp"
#include "boost/filesystem.hpp"
#include "get_graph.hpp"
#include "mapd.hpp"

namespace onlinedcoptaskplanner
{
OnlineDCOPTaskPlanner::OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name)
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
  std::vector<uint> final_config, final_waypoints;
  for (int i = 0; i < robot_number; i++)
  {
    final_config.push_back(waypoints[i].back());
    final_waypoints.push_back(waypoints_number[i]);
  }
  ROS_DEBUG_STREAM("final config: " << final_config);
  ROS_DEBUG_STREAM("robot_number: " << robot_number);

  // test reachability
  std::vector<std::vector<std::vector<bool> > > reachability(graph.size(), std::vector<std::vector<bool>>(robot_number));
  for (int j=0; j<graph.size(); j++)
  {
    for (int i=0; i<robot_number; i++)
    {
      reachability[j][i] = std::vector<bool>(waypoints_number[i], false);
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
      if (s.waypoint_indices[i] != waypoints_number[i] - 1 || s.configuration[i] != waypoints[i].back())
      {
        is_final = false;
        break;
      }
    }

    for (int i=0; i<robot_number; i++)
    {
      // update reachability
      uint v = s.configuration[i];
      reachability[v][i][s.waypoint_indices[i]] = true;
    }

    if (s.configuration == final_config)
    {
      ROS_DEBUG_STREAM(closed_count << "\tfinal config test - final config reached on these waypoints: " << s.waypoint_indices << "\tconfiguration: " << s.configuration << "\tfinal config: " << final_config);
    }

    if (s.waypoint_indices == final_waypoints)
    {
      ROS_DEBUG_STREAM(closed_count << "\tfinal waypoints test - last waypoints reached in this configuration " << s.configuration);
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
  for (int i=0; i<reachability.size(); i++)
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
    total_states *= x;
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
  else if (removed_edges.size())
  {
    // insert removed edges inside token to notify the agents
    for (const logistic_sim::Edge &e : removed_edges)
    {
      token.REMOVED_EDGES.push_back(e);
    }
    removed_edges.clear();
    edges_mutex.unlock();

    bool property_validity = check_conflict_free_property();
    ROS_ERROR_STREAM_COND(!property_validity, "Conflict-free property does not hold anymore!");
    ROS_INFO_STREAM_COND(property_validity, "Conflict-free property still holds true");

    token.NEED_REPAIR = true;
    token.HAS_REPAIRED_PATH = std::vector<uint8_t>(TEAM_SIZE, false);
  }
  else if (msg->REPAIR)
  {
    edges_mutex.unlock();
    ROS_DEBUG_STREAM("Waiting for agents to repair their paths...");
  }
  else if (msg->MULTI_PLAN_REPAIR)
  {
    edges_mutex.unlock();
    ROS_WARN_STREAM("For now we use centralized implementation of multi-robot repair!");
    // at this point, we have the robots' waypoints and their current position inside the token
    // run MAPD algorithm to find a solution and insert such solution inside the token

    multi_agent_repair(msg, token);
  }
  else
  {
    edges_mutex.unlock();
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
      int cmd_result = system("./stop_experiment.sh");
    }
  }

  pub_token.publish(token);
  ros::spinOnce();
}

void OnlineDCOPTaskPlanner::init_token(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  token.HEADER.seq = 1;
  CAPACITY = msg->CAPACITY;
  token.INIT = false;
  token.END_SIMULATION = false;
  token.ALL_MISSIONS_INSERTED = false;
  if (!offline_mode)
  {
    ROS_DEBUG("online mode -- start time setted inside token init phase");
    start_time = ros::Time::now();
  }
  last_goal_time = ros::Time::now();
  last_goal_status = std::vector<unsigned int>(TEAM_SIZE, 0);
}

void OnlineDCOPTaskPlanner::multi_agent_repair(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  // get valid paths
  std::vector<std::vector<uint>> other_paths;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (msg->HAS_REPAIRED_PATH[i])
    {
      std::vector<uint> robot_path = msg->TRAILS[i].PATH;
      robot_path.insert(robot_path.end(), msg->HOME_TRAILS[i].PATH.begin(), msg->HOME_TRAILS[i].PATH.end());
      other_paths.push_back(robot_path);
    }
  }

  // build map graph
  map_graph = build_graph();
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

  // test with no other paths
  astar_search_function(waypoints, initial_state, map_graph, robot_ids, &h_func, std::vector<std::vector<uint>>(),
                        &paths, &home_paths, going_home);
  // search_function(waypoints, initial_state, map_graph, robot_ids, &h_func, other_paths, &paths, &home_paths,
  // going_home);

  // insert paths inside token
  for (int i = 0; i < paths.size(); i++)
  {
    uint robot_id = robot_ids[i];
    token.TRAILS[robot_id].PATH.insert(token.TRAILS[robot_id].PATH.end(), paths[i].begin(), paths[i].end());
    token.HOME_TRAILS[robot_id].PATH = home_paths[i];
    ROS_DEBUG_STREAM("final task path: " << paths[i]);
    ROS_DEBUG_STREAM("final home path: " << home_paths[i]);
  }

  token.HAS_REPAIRED_PATH = std::vector<uint8_t>(TEAM_SIZE, true);
  token.MULTI_PLAN_REPAIR = false;
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

bool OnlineDCOPTaskPlanner::change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res)
{
  int result;

  if (msg.remove)
  {
    ROS_INFO_STREAM("Requested remotion of edge (" << msg.u << "," << msg.v << ")");
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
    ROS_ERROR_STREAM_COND(!result, "Failed addition!");

    if (result)
    {
      ROS_INFO_STREAM("TODO EDGE ADDITION NOTIFICATION");
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
  uint g, f;
  int prev;

  conflict_free_state() : g(0), f(0), prev(-1)
  {
  }

  bool operator<(const conflict_free_state &other)
  {
    if (f < other.f)
      return true;
    else if (g > other.g)
      return true;
    return false;
  }
};

// implements tree search but tells which robots can reach a particular vertex from home
// for now no heuristic
std::vector<bool> OnlineDCOPTaskPlanner::_check_conflict_free_impl(uint task_endpoint)
{
  int found_count = 0;
  std::vector<bool> result(TEAM_SIZE);
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
    auto it = std::find(home_vertex.begin(), home_vertex.end(), u_id);
    if (it != home_vertex.end())
    {
      int pos = it - home_vertex.begin();
      // ROS_DEBUG_STREAM(pos);
      result[pos] = true;
      found_count++;
      if (found_count == TEAM_SIZE)
        return result;
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

bool OnlineDCOPTaskPlanner::check_conflict_free_property()
{
  std::vector<bool> property_validity(TEAM_SIZE, true);
  std::vector<uint> task_endpoints = dst_vertex;
  task_endpoints.push_back(src_vertex);
  for (uint v : task_endpoints)
  {
    auto result = _check_conflict_free_impl(v);
    ROS_DEBUG_STREAM("Vertex " << v << ":\t");
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      bool r = result[i];
      ROS_DEBUG_STREAM_COND(r, " Y");
      ROS_DEBUG_STREAM_COND(!r, " N");
      if (!r)
      {
        property_validity[i] = false;
      }
    }
  }

  for (bool r : property_validity)
  {
    if (r)
      return true;
  }
  return false;
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

  if (signal(SIGSEGV, crash_handler) == SIG_ERR)
  {
    std::cerr << "can't print stack trace" << std::endl;
  }
  ODTP.run();
  return 0;
}