#include "OnlineDCOPTaskPlanner.hpp"
#include "boost/filesystem.hpp"
#include "get_graph.hpp"

namespace onlinedcoptaskplanner
{

OnlineDCOPTaskPlanner::OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name)
  : OnlineTaskPlanner(nh_, name)
{

}


void OnlineDCOPTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{

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
    ROS_INFO_STREAM("TODO DCOP TOKEN");
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


void OnlineDCOPTaskPlanner::advertise_change_edge_service(ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM("Subscribing to change_edge service");
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


bool OnlineDCOPTaskPlanner::change_edge(logistic_sim::ChangeEdge::Request &msg, logistic_sim::ChangeEdge::Response &res)
{
  bool result;

  if (msg.remove)
  {
    ROS_INFO_STREAM("Requested remotion of edge (" << msg.u << "," << msg.v << ")");
    result = RemoveEdge(vertex_web, dimension, msg.u, msg.v);
    ROS_ERROR_STREAM_COND(!result, "Failed remotion!");

    if (result)
    {
      ROS_INFO_STREAM("TODO EDGE REMOVAL NOTIFICATION");
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


} // namespace onlinedcoptaskplanner

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinedcoptaskplanner::OnlineDCOPTaskPlanner ODTP(nh_);
  ODTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ODTP.run();
  return 0;
}