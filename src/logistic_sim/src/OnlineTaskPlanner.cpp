#include <chrono>
#include "OnlineTaskPlanner.hpp"
#include "algorithms.hpp"
#include "boost/filesystem.hpp"
#include "partition.hpp"

namespace onlinetaskplanner
{
OnlineTaskPlanner::OnlineTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
{
  window_size = 11;
  
  sub_token = nh_.subscribe("token", 1, &OnlineTaskPlanner::token_callback, this);
  pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

  nh_.setParam("/simulation_running", "true");
}

// void OnlineTaskPlanner::run()
// {
//   std::cout << "Generating mission windows" << std::endl;
//   while (!missions.empty())
//   {
//     /*
//     * in this loop single-item tasks are divided in windows
//     * and each window is aggregated to form multi-item tasks windows
//     */
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
void OnlineTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
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

// std::vector<logistic_sim::Mission> OnlineTaskPlanner::set_partition(const std::vector<logistic_sim::Mission> &ts)
// {
//   auto start = std::chrono::system_clock::now();
//   // c_print("Calculating partitions", green, P);
//   ROS_INFO_STREAM("Calculating partitions");
//   std::vector<t_coalition> good_partition;
//   try
//   {
//     int num_tasks = ts.size();
//     // c_print(num_tasks);
//     partition::iterator it(num_tasks);
//     static int id_partition = 0;

//     t_coalition candidate;
//     while (true)
//     {
//       std::vector<std::vector<logistic_sim::Mission>> partitions = *it[ts];
//       auto n_subsets = it.subsets();
//       logistic_sim::Mission candidate_partition;
//       candidate_partition.ID = id_partition;
//       // c_print(id_partition);
//       id_partition++;
//       int id_subset = 0;
//       double V = 0;
//       candidate.second = candidate_partition;
//       std::vector<logistic_sim::Mission> m;
//       for (int i = 0; i < n_subsets; i++)
//       {
//         std::vector<logistic_sim::Mission> subset = partitions[i];
//         logistic_sim::Mission candidate_subset;
//         candidate_subset.ID = id_subset;
//         id_subset++;
//         for (int j = 0; j < subset.size(); j++)
//         {
//           candidate_subset.TOT_DEMAND += subset[j].TOT_DEMAND;
//           copy(subset[j].DEMANDS.begin(), subset[j].DEMANDS.end(), back_inserter(candidate_subset.DEMANDS));
//           copy(subset[j].DSTS.begin(), subset[j].DSTS.end(), back_inserter(candidate_subset.DSTS));
//           copy(subset[j].ITEM.begin(), subset[j].ITEM.end(), back_inserter(candidate_subset.ITEM));
//         }

//         // removing doubles from DSTS
//         for (int j = 0; j < candidate_subset.DSTS.size() - 1; j++)
//         {
//           if (candidate_subset.DSTS[j] == candidate_subset.DSTS[j + 1])
//           {
//             candidate_subset.DSTS.erase(candidate_subset.DSTS.begin() + j + 1);
//             j--;
//           }
//         }

//         // calculate mission path, needed to find the V value
//         std::vector<uint> path;
//         int dijkstra_result[64];
//         uint dijkstra_size;
//         dijkstra(src_vertex, *candidate_subset.DSTS.begin(), dijkstra_result, dijkstra_size, vertex_web, dimension);
//         path.insert(path.end(), dijkstra_result, dijkstra_result + dijkstra_size);
//         for (auto it = candidate_subset.DSTS.begin(); it + 1 != candidate_subset.DSTS.end(); it++)
//         {
//           dijkstra(*it, *(it + 1), dijkstra_result, dijkstra_size, vertex_web, dimension);
//           path.pop_back();
//           path.insert(path.end(), dijkstra_result, dijkstra_result + dijkstra_size);
//         }
//         candidate_subset.PATH_DISTANCE = compute_cost_of_route(path);
//         candidate_subset.V = (double)candidate_subset.PATH_DISTANCE / (double)candidate_subset.TOT_DEMAND;

//         candidate.second.V += candidate_subset.V;

//         if (candidate_subset.TOT_DEMAND > TEAM_CAPACITY)
//         {
//           candidate.second.GOOD++;
//         }
//         m.push_back(candidate_subset);
//       }

//       ++it;
//       candidate.first = m;
//       if (candidate.second.GOOD == 0)
//       {
//         // c_print("ok", green);
//         // print_coalition(*it);
//         good_partition.push_back(candidate);
//       }
//     }
//   }
//   catch (std::overflow_error &)
//   {
//   }

//   std::sort(good_partition.begin(), good_partition.end(), less_V());

//   t_coalition ele;
//   if (!good_partition.empty())
//   {
//     ele = good_partition.front();
//   }
//   else
//   {
//     ele.first = ts;
//     ele.second = logistic_sim::Mission();
//   }

//   // print_coalition(ele);

//   auto end = std::chrono::system_clock::now();
//   std::chrono::duration<double> elapsed_seconds = end - start;
//   times_file << "aggregation:\t" << elapsed_seconds.count() << "\n";
//   times_file.flush();
//   return ele.first;
// }

bool OnlineTaskPlanner::check_paths_conflicts(const std::vector<logistic_sim::Path> &paths, bool print)
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
          ss << "[WARN] Robot " << ri << " and " << rj << " will meet in edge (" << p1.PATH[i] << "," << p1.PATH[i + 1]
             << ") at timestep " << i;
          c_print(ss.str(), yellow, print);
        }
      }
    }
  }

  if (conflicts > 0)
  {
    c_print("[WARN] ", conflicts, " conflicts detected", yellow, print);
  }

  return good;
}

// void OnlineTaskPlanner::write_simple_missions(std::ostream &os, const std::vector<logistic_sim::Mission> &missions)
// {
//     os << missions.size() << "\n\n";
//     for (const logistic_sim::Mission &m : missions)
//     {
//         os << m.ID << "\n";
//         uint dst = m.DSTS[0];
//         // find index in DSTS vector
//         auto it = std::find(dst_vertex.begin(), dst_vertex.end(), dst);
//         os << it - dst_vertex.begin() << "\n";

//         uint dms = m.DEMANDS[0];
//         os << dms << "\n";

//         os << "\n";
//     }

//     os << std::flush;
// }

// std::vector<logistic_sim::Mission> OnlineTaskPlanner::read_simple_missions(std::istream &is)
// {
//     std::vector<logistic_sim::Mission> missions;
//     int n_missions;
//     is >> n_missions;

//     for (int i = 0; i < n_missions; i++)
//     {
//         logistic_sim::Mission m;
//         is >> m.ID;
//         uint dst_index;
//         is >> dst_index;
//         m.DSTS.push_back(dst_vertex[dst_index]);

//         uint dms;
//         is >> dms;
//         m.DEMANDS.push_back(dms);

//         // calculate path and V metric
//         copy(paths[dst_index].begin(), paths[dst_index].end(), back_inserter(m.ROUTE));
//         m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
//         m.TOT_DEMAND = dms;
//         m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

//         missions.push_back(m);
//     }

//     return missions;
// }

void OnlineTaskPlanner::allocate_memory()
{
  // this task planner does not require heap memory
}

}  // namespace onlinetaskplanner

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinetaskplanner::OnlineTaskPlanner OTP(nh_);
  OTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  OTP.run();
  return 0;
}