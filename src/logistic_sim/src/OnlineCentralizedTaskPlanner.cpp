#include "OnlineCentralizedTaskPlanner.hpp"
#include "partition.hpp"
#include "algorithms.hpp"
#include "boost/filesystem.hpp"
#include <memory>
#include <chrono>

namespace onlinecentralizedtaskplanner
{

bool astar_cmp_function(const std::vector<std::vector<unsigned int>> &min_hops_matrix,
                        const std::vector<unsigned int> &waypoints, const st_location &lhs, const st_location &rhs)
{
  // uso la min_hops_matrix come euristica sulla lunghezza del percorso rimanente
  unsigned int lhs_h = min_hops_matrix[lhs.vertex][waypoints[lhs.waypoint]];
  unsigned int rhs_h = min_hops_matrix[rhs.vertex][waypoints[rhs.waypoint]];
  // TEST
  // std::cout << "[DEBUG]\tVertex: " << rhs.vertex << "\tTime: " << rhs.time
  //           << "\tWaypoint: " << waypoints[rhs.waypoint]
  //           << "\tf-value: " << rhs.time + rhs_h << std::endl;
  if (lhs.time + lhs_h < rhs.time + rhs_h)
  {
    return true;
  }
  // else if (lhs.time + lhs_h == rhs.time + rhs_h)
  // {
  //   if (lhs.waypoint > rhs.waypoint)
  //   {
  //     return true;
  //   }
  // }

  return false;
}

// void print_coalition(const t_coalition &coalition)
// {
//   auto tasks = coalition.first;
//   auto mission = coalition.second;
//   c_print("Mission id: ", mission.ID, green, P);
//   // std::cout << "pd: " << mission.PATH_DISTANCE << "\n";
//   std::cout << "td: " << mission.TOT_DEMAND << "\n";
//   // std::cout << " V: " << mission.V << "\n";
//   // auto size_dsts = mission.DSTS.size();
//   // std::cout << "dsts"
//   //           << "\n";
//   // for (int i = 0; i < size_dsts; i++)
//   // {
//   //     std::cout << mission.DSTS[i] << " ";
//   // }
//   // std::cout << "\n";

//   auto size_boh = mission.DEMANDS.size();
//   std::cout << "demands"
//             << "\n";
//   for (int i = 0; i < size_boh; i++)
//   {
//     std::cout << mission.DEMANDS[i] << " ";
//   }
//   std::cout << "\n";

//   // auto size_route = mission.ROUTE.size();
//   // std::cout << "route"
//   //           << "\n";
//   // for (int i = 0; i < size_route; i++)
//   // {
//   // std::cout << mission.ROUTE[i] << " ";
//   // }
//   // std::cout << "\n";

//   // c_print("tasks", magenta, P);

//   auto size_tasks = tasks.size();

//   for (auto i = 0; i < size_tasks; i++)
//   {
//     auto t = tasks[i];
//     c_print("task id: ", t.ID, magenta, P);
//     //     std::cout << "pd: " << t.PATH_DISTANCE << "\n";
//     std::cout << "td: " << t.TOT_DEMAND << "\n";
//     //     std::cout << " V: " << t.V << "\n";
//     auto size_dsts = t.DSTS.size();
//     std::cout << "dsts"
//               << "\n";
//     for (int i = 0; i < size_dsts; i++)
//     {
//       std::cout << t.DSTS[i] << " ";
//     }
//     std::cout << "\n";

//     auto size_boh = t.DEMANDS.size();
//     std::cout << "demands"
//               << "\n";
//     for (int i = 0; i < size_boh; i++)
//     {
//       std::cout << t.DEMANDS[i] << " ";
//     }
//     std::cout << "\n";

//     //     auto size_route = t.ROUTE.size();
//     //     std::cout << "route"
//     //               << "\n";
//     //     for (int i = 0; i < size_route; i++)
//     //     {
//     //         std::cout << t.ROUTE[i] << " ";
//     //     }
//     //     std::cout << "\n";
//   }

//   // c_print("fine mission id: ", mission.ID, red, P);
// }

OnlineCentralizedTaskPlanner::OnlineCentralizedTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
{
  window_size = 4;

  sub_token = nh_.subscribe("token", 1, &OnlineCentralizedTaskPlanner::token_callback, this);
  pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

  nh_.setParam("/simulation_running", "true");
}

// void OnlineCentralizedTaskPlanner::run()
// {
//   std::cout << "Generating mission windows" << std::endl;
//   while (!missions.empty())
//   {
//     /*
//      * in this loop single-item tasks are divided in windows
//      * and each window is aggregated to form multi-item tasks windows
//      */
//     auto first_it = missions.begin();
//     auto last_it = missions.end();
//     if (missions.size() >= window_size)
//     {
//       last_it = first_it + window_size;
//     }
//     std::vector<logistic_sim::Mission> tasks(first_it, last_it);
//     missions.erase(first_it, last_it);
//     std::vector<logistic_sim::Mission> window = set_partition(tasks);
//     // mission_windows is accessed also in the token callback thread, hence the mutex
//     window_mutex.lock();
//     mission_windows.push_back(window);
//     c_print("New window aggregated - to be inserted into token: ", mission_windows.size(), yellow, P);
//     c_print("Tasks not yet aggregated: ", missions.size(), green, P);
//     c_print("");
//     window_mutex.unlock();
//   }

//   // after computing the last window we wait for shutdown
//   ros::waitForShutdown();
// }

/*
 * handles statistics (as all task planners)
 * handles insertion of new tasks in the token
 *
 * the NEW_MISSIONS_AVAILABLE is set to true to signal to agent
 * that new missions have been added
 *
 */
void OnlineCentralizedTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{
  // c_print("From: ", msg->ID_SENDER, yellow, P);
  // c_print("To: ", msg->ID_RECEIVER, yellow, P);
  // c_print("");

  static int last_mission_size = 0;
  if (msg->ID_RECEIVER != TASK_PLANNER_ID)
    return;

  logistic_sim::Token token;
  token = *msg;
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;
  if (msg->INIT)
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
    last_mission_size = 0;
  }
  else
  {
    token.HEADER.seq += 1;

    // prints remaining tasks
    // if (token.MISSION.size() != last_mission_size)
    // {
    //     last_mission_size = token.MISSION.size();
    //     c_print("Remaining tasks: ", last_mission_size, green);
    // }

    // the mutex is necessary because the window is updated in another thread
    window_mutex.lock();
    // signals to the robots that new missions are in the token
    // robots will set this flag to false when this missions have been accepted
    if (!mission_windows.empty() && !token.NEW_MISSIONS_AVAILABLE)
    {
      token.NEW_MISSIONS_AVAILABLE = true;
    }
    // when all robots are synced it is possible to plan the allocation of the new tasks
    else if (!mission_windows.empty() && token.SYNCED_ROBOTS)
    {
      //   token.MISSION = mission_windows.front();
      //   token.TAKEN_MISSION = std::vector<int>(token.MISSION.size(), -1);
      path_partition(token, mission_windows.front());
      mission_windows.pop_front();
      // signals in the token when all the missions have been inserted
      if (missions.empty())
      {
        token.ALL_MISSIONS_INSERTED = true;
      }
      c_print("Mission window inserted in token - remaining windows: ", mission_windows.size(), yellow, P);
      token.SYNCED_ROBOTS = false;
      token.NEW_MISSIONS_AVAILABLE = false;
      if (offline_mode)
      {
        ROS_DEBUG("offline mode -- start time setted after allocation");
        start_time = ros::Time::now();
      }
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

void OnlineCentralizedTaskPlanner::build_map_graph()
{
  allocate_memory();
  map_graph = std::vector<std::vector<unsigned int>>(dimension);
  for (int i = 0; i < dimension; i++)
  {
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      map_graph[i].push_back(vertex_web[i].id_neigh[j]);
    }
  }
  min_hops_matrix = calculate_min_hops_matrix();
}

/*
 * Floyd-Warshall to find the least number of hops between each pair of vertices
 */
std::vector<std::vector<unsigned int>> OnlineCentralizedTaskPlanner::calculate_min_hops_matrix()
{
  unsigned int infinity = std::numeric_limits<unsigned int>::max();
  std::vector<std::vector<unsigned int>> result(dimension,
                                                // std::vector<unsigned int>(dimension, 0));
                                                std::vector<unsigned int>(dimension, infinity));
  for (unsigned int u = 0; u < dimension; u++)
  {
    // self loop is 1 because the robot must wait for the others to do their moves
    result[u][u] = 1;
    for (unsigned int v : map_graph[u])
    {
      result[u][v] = 1;
    }
  }

  for (unsigned int k = 0; k < dimension; k++)
  {
    for (unsigned int i = 0; i < dimension; i++)
    {
      for (unsigned int j = 0; j < dimension; j++)
      {
        if (result[i][k] != infinity && result[k][j] != infinity && result[i][j] > result[i][k] + result[k][j])
        {
          result[i][j] = result[i][k] + result[k][j];
        }
      }
    }
  }

  return result;
}

bool OnlineCentralizedTaskPlanner::check_paths_conflicts(const std::vector<logistic_sim::Path> &paths, bool print)
{
  bool good = true;
  int conflicts = 0;

  for (auto it = paths.begin(); it != paths.end(); it++)
  {
    for (auto jt = it + 1; jt != paths.end(); jt++)
    {
      int ri = it - paths.begin();
      int rj = jt - paths.begin();
      const logistic_sim::Path &p1 = *it;
      const logistic_sim::Path &p2 = *jt;
      int n = std::min(p1.PATH.size(), p2.PATH.size());
      for (int i = 0; i < n - 1; i++)
      {
        if (p1.PATH[i] == p2.PATH[i])
        {
          conflicts++;
          good = false;
          std::stringstream ss;
          ss << "[WARN] Robot " << ri << " and " << rj << " will meet in vertex " << p1.PATH[i] << " at timestep " << i;
          c_print(ss.str(), yellow, print);
        }
        if (p1.PATH[i] == p2.PATH[i + 1] && p2.PATH[i] == p1.PATH[i + 1])
        {
          conflicts++;
          good = false;
          std::stringstream ss;
          ss << "[WARN] Robot " << ri << " and " << rj << " will meet at edge (" << p1.PATH[i] << ","
             << p1.PATH[i + 1] << ") in timestep " << i;
          c_print(ss.str(), yellow, print);
        }
      }
    }
  }

  if (conflicts > 0)
  {
    c_print("[WARN] ", conflicts, "conflicts detected", yellow, print);
  }

  return good;
}

// void OnlineCentralizedTaskPlanner::write_simple_missions(std::ostream &os,
//                                                          const std::vector<logistic_sim::Mission> &missions)
// {
//   os << missions.size() << "\n\n";
//   for (const logistic_sim::Mission &m : missions)
//   {
//     os << m.ID << "\n";
//     uint dst = m.DSTS[0];
//     // find index in DSTS vector
//     auto it = std::find(dst_vertex.begin(), dst_vertex.end(), dst);
//     os << it - dst_vertex.begin() << "\n";

//     uint dms = m.DEMANDS[0];
//     os << dms << "\n";

//     os << "\n";
//   }

//   os << std::flush;
// }

// std::vector<logistic_sim::Mission> OnlineCentralizedTaskPlanner::read_simple_missions(std::istream &is)
// {
//   std::vector<logistic_sim::Mission> missions;
//   int n_missions;
//   is >> n_missions;

//   for (int i = 0; i < n_missions; i++)
//   {
//     logistic_sim::Mission m;
//     is >> m.ID;
//     uint dst_index;
//     is >> dst_index;
//     m.DSTS.push_back(dst_vertex[dst_index]);

//     uint dms;
//     is >> dms;
//     m.DEMANDS.push_back(dms);

//     // calculate path and V metric
//     copy(paths[dst_index].begin(), paths[dst_index].end(), back_inserter(m.ROUTE));
//     m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
//     m.TOT_DEMAND = dms;
//     m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

//     missions.push_back(m);
//   }

//   return missions;
// }

// void OnlineCentralizedTaskPlanner::real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot)
// {
//   last_real_pos[id_robot] = *msg;
// }

// void OnlineCentralizedTaskPlanner::amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg,
//                                                      int id_robot)
// {
//   last_amcl_pos[id_robot] = *msg;
//   nav_msgs::Odometry *real_pos = &last_real_pos[id_robot];

//   double distance = sqrt((msg->pose.pose.position.x - real_pos->pose.pose.position.x) *
//                              (msg->pose.pose.position.x - real_pos->pose.pose.position.x) +
//                          (msg->pose.pose.position.y - real_pos->pose.pose.position.y) *
//                              (msg->pose.pose.position.y - real_pos->pose.pose.position.y));
//   ROS_DEBUG_STREAM("Robot " << id_robot << " position error: " << distance);
//   robot_pos_errors[id_robot].push_back(distance);
// }

} // namespace onlinecentralizedtaskplanner