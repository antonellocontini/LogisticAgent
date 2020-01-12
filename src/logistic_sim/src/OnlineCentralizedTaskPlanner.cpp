#include "OnlineCentralizedTaskPlanner.hpp"
#include "partition.hpp"
#include "algorithms.hpp"
#include "boost/filesystem.hpp"
#include <memory>

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

void print_coalition(const t_coalition &coalition)
{
  auto tasks = coalition.first;
  auto mission = coalition.second;
  c_print("Mission id: ", mission.ID, green, P);
  // std::cout << "pd: " << mission.PATH_DISTANCE << "\n";
  std::cout << "td: " << mission.TOT_DEMAND << "\n";
  // std::cout << " V: " << mission.V << "\n";
  // auto size_dsts = mission.DSTS.size();
  // std::cout << "dsts"
  //           << "\n";
  // for (int i = 0; i < size_dsts; i++)
  // {
  //     std::cout << mission.DSTS[i] << " ";
  // }
  // std::cout << "\n";

  auto size_boh = mission.DEMANDS.size();
  std::cout << "demands"
            << "\n";
  for (int i = 0; i < size_boh; i++)
  {
    std::cout << mission.DEMANDS[i] << " ";
  }
  std::cout << "\n";

  // auto size_route = mission.ROUTE.size();
  // std::cout << "route"
  //           << "\n";
  // for (int i = 0; i < size_route; i++)
  // {
  // std::cout << mission.ROUTE[i] << " ";
  // }
  // std::cout << "\n";

  // c_print("tasks", magenta, P);

  auto size_tasks = tasks.size();

  for (auto i = 0; i < size_tasks; i++)
  {
    auto t = tasks[i];
    c_print("task id: ", t.ID, magenta, P);
    //     std::cout << "pd: " << t.PATH_DISTANCE << "\n";
    std::cout << "td: " << t.TOT_DEMAND << "\n";
    //     std::cout << " V: " << t.V << "\n";
    auto size_dsts = t.DSTS.size();
    std::cout << "dsts"
              << "\n";
    for (int i = 0; i < size_dsts; i++)
    {
      std::cout << t.DSTS[i] << " ";
    }
    std::cout << "\n";

    auto size_boh = t.DEMANDS.size();
    std::cout << "demands"
              << "\n";
    for (int i = 0; i < size_boh; i++)
    {
      std::cout << t.DEMANDS[i] << " ";
    }
    std::cout << "\n";

    //     auto size_route = t.ROUTE.size();
    //     std::cout << "route"
    //               << "\n";
    //     for (int i = 0; i < size_route; i++)
    //     {
    //         std::cout << t.ROUTE[i] << " ";
    //     }
    //     std::cout << "\n";
  }

  // c_print("fine mission id: ", mission.ID, red, P);
}

OnlineCentralizedTaskPlanner::OnlineCentralizedTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
{
  sub_token = nh_.subscribe("token", 1, &OnlineCentralizedTaskPlanner::token_callback, this);
  pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

  nh_.setParam("/simulation_running", "true");
}

void OnlineCentralizedTaskPlanner::init(int argc, char **argv)
{
  srand(time(NULL));
  int cmd_result = chdir(PS_path.c_str());
  mapname = string(argv[1]);
  std::string graph_file = "maps/" + mapname + "/" + mapname + ".graph";
  dimension = GetGraphDimension(graph_file.c_str());
  vertex_web = new vertex[dimension];
  GetGraphInfo(vertex_web, dimension, graph_file.c_str());
  uint nedges = GetNumberEdges(vertex_web, dimension);
  printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);

  ALGORITHM = argv[2];
  TEAM_SIZE = atoi(argv[3]);
  GENERATION = argv[4];
  TEAM_CAPACITY = atoi(argv[5]);

  // task set file name (if generation = FILE)
  task_set_file = argv[6];

  src_vertex = map_src[mapname];
  dst_vertex = map_dsts[mapname];

  // building paths for task aggregation euristic
  for (uint dst : dst_vertex)
  {
    int result[100];
    uint result_size;
    dijkstra(src_vertex, dst, result, result_size, vertex_web, dimension);
    paths.push_back(std::vector<uint>(result, result + result_size));
  }

  // allocate memory for path planning algorithm
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

  // starting service to check when all robots are ready
  robots_ready_status = std::vector<bool>(TEAM_SIZE);
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("robot_ready", &OnlineCentralizedTaskPlanner::robot_ready, this);
  ros::spinOnce();

  // subscribe to robot's real and amcl position (to measure error)
  last_real_pos = std::vector<nav_msgs::Odometry>(TEAM_SIZE);
  last_amcl_pos = std::vector<geometry_msgs::PoseWithCovarianceStamped>(TEAM_SIZE);
  robot_pos_errors = std::vector<std::vector<double>>(TEAM_SIZE);
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    std::stringstream real_ss, amcl_ss;
    real_ss << "/robot_" << i << "/base_pose_ground_truth";
    amcl_ss << "/robot_" << i << "/amcl_pose";
    ros::Subscriber real_sub = nh.subscribe<nav_msgs::Odometry>(
        real_ss.str(), 10, boost::bind(&OnlineCentralizedTaskPlanner::real_pos_callback, this, _1, i));
    ros::Subscriber amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        amcl_ss.str(), 10, boost::bind(&OnlineCentralizedTaskPlanner::amcl_pos_callback, this, _1, i));

    if (real_sub)
    {
      real_pos_sub.push_back(real_sub);
      ROS_INFO_STREAM("Subscribed to " << real_ss.str());
    }
    else
    {
      ROS_WARN_STREAM("Can't subscribe to " << real_ss.str());
    }

    if (amcl_sub)
    {
      amcl_pos_sub.push_back(amcl_sub);
      ROS_INFO_STREAM("Subscribed to " << amcl_ss.str());
    }
    else
    {
      ROS_WARN_STREAM("Can't subscribe to " << amcl_ss.str());
    }
  }

  allocate_memory();
  if (GENERATION != "file")
  {
    missions_generator(GENERATION);
    std::stringstream conf_dir_name;
    conf_dir_name << "missions";
    //<< name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << TEAM_SIZE << "capacity" << TEAM_CAPACITY << "_"
    //<< mapname;
    boost::filesystem::path conf_directory(conf_dir_name.str());
    if (!boost::filesystem::exists(conf_directory))
    {
      boost::filesystem::create_directory(conf_directory);
    }

    std::string filename;
    int run_number = 1;
    std::stringstream filenamestream;
    std::ifstream check_new;
    // loop per controllare se il file già esiste
    do
    {
      filenamestream.str("");  // cancella la stringa
      filenamestream << conf_dir_name.str() << "/" << run_number << ".txt";
      check_new = std::ifstream(filenamestream.str());
      run_number++;
    } while (check_new);
    check_new.close();
    filename = filenamestream.str();
    std::ofstream file(filename);
    write_simple_missions(file, missions);
  }
  else
  {
    c_print("Reading missions from file...", green, P);
    std::stringstream taskset_dir_name;
    taskset_dir_name << "missions/" << task_set_file;
    std::string filename(taskset_dir_name.str());
    boost::filesystem::path missions_path(filename);
    if (!boost::filesystem::exists(missions_path))
    {
      c_print("Missions file ", filename, " does not exists!!!", red, P);
      sleep(2);
      ros::shutdown();
      int cmd_result = system("./stop_experiment.sh");
    }
    ifstream missions_file(filename);
    // missions_file >> missions;
    missions = read_simple_missions(missions_file);
    nTask = missions.size();
    missions_file.close();
  }

  c_print("Tasks number: ", missions.size(), green, P);
  // print missions
  // std::cout << "Single item task-set:\n";
  // for (const logistic_sim::Mission &m : missions)
  // {
  //     std::cout << "ID: " << m.ID << "\n";
  //     std::cout << "DEMANDS:\n";
  //     for (auto v : m.DEMANDS)
  //     {
  //         std::cout << v << " ";
  //     }
  //     std::cout << "DSTS:\n";
  //     for (auto v : m.DSTS)
  //     {
  //         std::cout << v << " ";
  //     }
  //     std::cout << std::endl;
  // }

  // initializing stats structure
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    taskplanner::MonitorData data;
    robots_data.push_back(data);
  }

  // waiting for agents to be ready
  while (robots_ready_count < TEAM_SIZE)
  {
    ros::Duration(1, 0).sleep();
    ros::spinOnce();
  }

  // wait
  ros::Duration(3.0).sleep();

  logistic_sim::Token token;

  // initialization round
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;
  token.INIT = true;
  token.NEW_MISSIONS_AVAILABLE = false;
  token.SHUTDOWN = false;

  pub_token.publish(token);
  ros::spinOnce();

  sleep(1);

  c_print("INIT", green);
}

void OnlineCentralizedTaskPlanner::run()
{
  std::cout << "Generating mission windows" << std::endl;
  while (!missions.empty())
  {
    /*
     * in this loop single-item tasks are divided in windows
     * and each window is aggregated to form multi-item tasks windows
     */
    auto first_it = missions.begin();
    auto last_it = missions.end();
    if (missions.size() >= window_size)
    {
      last_it = first_it + window_size;
    }
    std::vector<logistic_sim::Mission> tasks(first_it, last_it);
    missions.erase(first_it, last_it);
    std::vector<logistic_sim::Mission> window = set_partition(tasks);
    // mission_windows is accessed also in the token callback thread, hence the mutex
    window_mutex.lock();
    mission_windows.push_back(window);
    c_print("New window aggregated - to be inserted into token: ", mission_windows.size(), yellow, P);
    c_print("Tasks not yet aggregated: ", missions.size(), green, P);
    c_print("");
    window_mutex.unlock();
  }

  // after computing the last window we wait for shutdown
  ros::waitForShutdown();
}

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
    start_time = ros::Time::now();
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

std::vector<logistic_sim::Mission> OnlineCentralizedTaskPlanner::set_partition(const std::vector<logistic_sim::Mission> &ts)
{
  c_print("Calculating partitions", green, P);
  std::vector<t_coalition> good_partition;
  try
  {
    int num_tasks = ts.size();
    // c_print(num_tasks);
    partition::iterator it(num_tasks);
    static int id_partition = 0;

    t_coalition candidate;
    while (true)
    {
      std::vector<std::vector<logistic_sim::Mission>> partitions = *it[ts];
      auto n_subsets = it.subsets();
      logistic_sim::Mission candidate_partition;
      candidate_partition.ID = id_partition;
      // c_print(id_partition);
      id_partition++;
      int id_subset = 0;
      double V = 0;
      candidate.second = candidate_partition;
      std::vector<logistic_sim::Mission> m;
      for (int i = 0; i < n_subsets; i++)
      {
        std::vector<logistic_sim::Mission> subset = partitions[i];
        logistic_sim::Mission candidate_subset;
        candidate_subset.ID = id_subset;
        id_subset++;
        for (int j = 0; j < subset.size(); j++)
        {
          candidate_subset.TOT_DEMAND += subset[j].TOT_DEMAND;
          copy(subset[j].DEMANDS.begin(), subset[j].DEMANDS.end(), back_inserter(candidate_subset.DEMANDS));
          copy(subset[j].DSTS.begin(), subset[j].DSTS.end(), back_inserter(candidate_subset.DSTS));
          copy(subset[j].ITEM.begin(), subset[j].ITEM.end(), back_inserter(candidate_subset.ITEM));
        }

        // removing doubles from DSTS
        for (int j = 0; j < candidate_subset.DSTS.size() - 1; j++)
        {
          if (candidate_subset.DSTS[j] == candidate_subset.DSTS[j + 1])
          {
            candidate_subset.DSTS.erase(candidate_subset.DSTS.begin() + j + 1);
            j--;
          }
        }

        // calculate mission path, needed to find the V value
        std::vector<uint> path;
        int dijkstra_result[64];
        uint dijkstra_size;
        dijkstra(src_vertex, *candidate_subset.DSTS.begin(), dijkstra_result, dijkstra_size, vertex_web, dimension);
        path.insert(path.end(), dijkstra_result, dijkstra_result + dijkstra_size);
        for (auto it = candidate_subset.DSTS.begin(); it + 1 != candidate_subset.DSTS.end(); it++)
        {
          dijkstra(*it, *(it + 1), dijkstra_result, dijkstra_size, vertex_web, dimension);
          path.pop_back();
          path.insert(path.end(), dijkstra_result, dijkstra_result + dijkstra_size);
        }
        candidate_subset.PATH_DISTANCE = compute_cost_of_route(path);
        candidate_subset.V = (double)candidate_subset.PATH_DISTANCE / (double)candidate_subset.TOT_DEMAND;

        candidate.second.V += candidate_subset.V;

        if (candidate_subset.TOT_DEMAND > TEAM_CAPACITY)
        {
          candidate.second.GOOD++;
        }

        m.push_back(candidate_subset);
      }

      ++it;
      candidate.first = m;
      if (candidate.second.GOOD == 0)
      {
        // c_print("ok", green);
        // print_coalition(*it);
        good_partition.push_back(candidate);
      }
    }
  }
  catch (std::overflow_error &)
  {
  }

  std::sort(good_partition.begin(), good_partition.end(), less_V());

  t_coalition ele;
  if (!good_partition.empty())
  {
    ele = good_partition.front();
  }
  else
  {
    ele.first = ts;
    ele.second = logistic_sim::Mission();
  }

  // print_coalition(ele);

  return ele.first;
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

void OnlineCentralizedTaskPlanner::write_simple_missions(std::ostream &os,
                                                    const std::vector<logistic_sim::Mission> &missions)
{
  os << missions.size() << "\n\n";
  for (const logistic_sim::Mission &m : missions)
  {
    os << m.ID << "\n";
    uint dst = m.DSTS[0];
    // find index in DSTS vector
    auto it = std::find(dst_vertex.begin(), dst_vertex.end(), dst);
    os << it - dst_vertex.begin() << "\n";

    uint dms = m.DEMANDS[0];
    os << dms << "\n";

    os << "\n";
  }

  os << std::flush;
}

std::vector<logistic_sim::Mission> OnlineCentralizedTaskPlanner::read_simple_missions(std::istream &is)
{
  std::vector<logistic_sim::Mission> missions;
  int n_missions;
  is >> n_missions;

  for (int i = 0; i < n_missions; i++)
  {
    logistic_sim::Mission m;
    is >> m.ID;
    uint dst_index;
    is >> dst_index;
    m.DSTS.push_back(dst_vertex[dst_index]);

    uint dms;
    is >> dms;
    m.DEMANDS.push_back(dms);

    // calculate path and V metric
    copy(paths[dst_index].begin(), paths[dst_index].end(), back_inserter(m.ROUTE));
    m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
    m.TOT_DEMAND = dms;
    m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

    missions.push_back(m);
  }

  return missions;
}

void OnlineCentralizedTaskPlanner::real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot)
{
  last_real_pos[id_robot] = *msg;
}

void OnlineCentralizedTaskPlanner::amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg,
                                                int id_robot)
{
  last_amcl_pos[id_robot] = *msg;
  nav_msgs::Odometry *real_pos = &last_real_pos[id_robot];

  double distance = sqrt((msg->pose.pose.position.x - real_pos->pose.pose.position.x) *
                             (msg->pose.pose.position.x - real_pos->pose.pose.position.x) +
                         (msg->pose.pose.position.y - real_pos->pose.pose.position.y) *
                             (msg->pose.pose.position.y - real_pos->pose.pose.position.y));
  ROS_DEBUG_STREAM("Robot " << id_robot << " position error: " << distance);
  robot_pos_errors[id_robot].push_back(distance);
}

}  // namespace onlinecentralizedtaskplanner