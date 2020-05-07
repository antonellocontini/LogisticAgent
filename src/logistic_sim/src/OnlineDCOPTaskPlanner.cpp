#include "OnlineDCOPTaskPlanner.hpp"
#include "boost/filesystem.hpp"
#include "get_graph.hpp"
#include <unordered_map>

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

  edges_mutex.lock();
  if (msg->INIT)
  {
    edges_mutex.unlock();
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
  else if(removed_edges.size())
  {
    // insert removed edges inside token to notify the agents
    for (const logistic_sim::Edge &e : removed_edges)
    {
      token.REMOVED_EDGES.push_back(e);
    }
    removed_edges.clear();
    edges_mutex.unlock();
  }
  else
  {
    edges_mutex.unlock();
    // ROS_INFO_STREAM("TODO DCOP TOKEN");
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
  for (int i=0; i<dimension; i++)
  {
    test << vertex_web[i].id << std::endl;
    for (int j=0; j<vertex_web[i].num_neigh; j++)
    {
      test << "\t(" << vertex_web[i].id << "," << vertex_web[i].id_neigh[j] << ")" << std::endl;
    }
  }
  test.close();
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

      check_conflict_free_property();
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

  while(!open.empty())
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
      for (int i=0; i<vertex_web[u_id].num_neigh; i++)
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
  for (uint v : dst_vertex)
  {
    auto result = _check_conflict_free_impl(v);
    ROS_DEBUG_STREAM("Vertex " << v << ":\t");
    for (bool r : result)
    {
      ROS_DEBUG_STREAM_COND(r, " Y");
      ROS_DEBUG_STREAM_COND(!r, " N");
    }
  }
  auto result = _check_conflict_free_impl(src_vertex);
  ROS_DEBUG_STREAM("Vertex " << src_vertex << ":\t");
  for (bool r : result)
  {
    ROS_DEBUG_STREAM_COND(r, " Y");
    ROS_DEBUG_STREAM_COND(!r, " N");
  }
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