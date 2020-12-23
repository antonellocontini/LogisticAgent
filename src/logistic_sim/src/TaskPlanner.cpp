#include "TaskPlanner.hpp"

#include <xmlrpcpp/XmlRpcValue.h>
#include "algorithms.hpp"
#include "boost/filesystem.hpp"
#include "partition.hpp"

#include <chrono>

namespace taskplanner
{
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

  auto missions_number = mission.DEMANDS.size();
  std::cout << "demands"
            << "\n";
  for (int i = 0; i < missions_number; i++)
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

  c_print("fine mission id: ", mission.ID, red, P);
}

ostream &operator<<(ostream &os, const MonitorData &md)
{
  os << md.tot_distance << "," << md.interference_num << "," << md.completed_missions << "," << md.completed_tasks
     << "," << md.total_time << "," << md.total_steps;
  return os;
}

ostream &operator<<(ostream &os, const vector<MonitorData> &v)
{
  os << "ID_ROBOT,TOT_DISTANCE,INTERFERENCE_NUM,COMPLETED_MISSIONS,COMPLETED_TASKS,TOTAL_TIME,TOTAL_STEPS" << endl;
  for (int i = 0; i < v.size(); i++)
  {
    os << i << "," << v[i] << endl;
  }
  return os;
}

ostream &operator<<(ostream &os, const logistic_sim::Mission &m)
{
  os << m.ID << "\n";
  os << m.PRIORITY << "\n";

  os << m.ITEM.size() << "\n";
  for (auto &d : m.ITEM)
  {
    os << d << "\n";
  }

  os << m.ROUTE.size() << "\n";
  for (auto &d : m.ROUTE)
  {
    os << d << "\n";
  }

  os << m.DSTS.size() << "\n";
  for (auto &d : m.DSTS)
  {
    os << d << "\n";
  }

  os << m.DEMANDS.size() << "\n";
  for (auto &d : m.DEMANDS)
  {
    os << d << "\n";
  }

  os << m.V << "\n";
  os << m.TOT_DEMAND << "\n";
  os << m.PATH_DISTANCE << "\n";
  os << m.GOOD << "\n";

  os << flush;
  return os;
}

ostream &operator<<(ostream &os, const vector<logistic_sim::Mission> &v)
{
  os << v.size() << "\n\n";
  for (auto &m : v)
  {
    os << m << "\n";
  }

  os << flush;
  return os;
}

istream &operator>>(istream &is, logistic_sim::Mission &m)
{
  int n;

  is >> m.ID;
  is >> m.PRIORITY;

  // is >> m.ITEM.size();
  is >> n;
  m.ITEM.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.ITEM.push_back(v);
  }

  // is >> m.ROUTE.size();
  is >> n;
  m.ROUTE.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.ROUTE.push_back(v);
  }

  // is >> m.DSTS.size();
  is >> n;
  m.DSTS.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.DSTS.push_back(v);
  }

  // is >> m.DEMANDS.size();
  is >> n;
  m.DEMANDS.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.DEMANDS.push_back(v);
  }

  is >> m.V;
  is >> m.TOT_DEMAND;
  is >> m.PATH_DISTANCE;
  is >> m.GOOD;

  return is;
}

istream &operator>>(istream &is, vector<logistic_sim::Mission> &v)
{
  int n;
  is >> n;
  v.clear();

  for (int i = 0; i < n; i++)
  {
    logistic_sim::Mission m;
    is >> m;
    v.push_back(m);
  }
  return is;
}

bool operator==(const logistic_sim::Mission &lhs, const logistic_sim::Mission &rhs)
{
  auto cmp_double = [](double lhs, double rhs) {
    double diff = fabs(lhs - rhs);
    lhs = fabs(lhs);
    rhs = fabs(rhs);
    double largest = (rhs > lhs) ? rhs : lhs;
    if (diff <= largest * 25 * FLT_EPSILON)
      return true;
    return false;
  };

  if (lhs.ITEM.size() != rhs.ITEM.size())
    return false;

  for (int i = 0; i < lhs.ITEM.size(); i++)
    if (lhs.ITEM[i] != rhs.ITEM[i])
      return false;

  if (lhs.ROUTE.size() != rhs.ROUTE.size())
    return false;

  for (int i = 0; i < lhs.ROUTE.size(); i++)
    if (lhs.ROUTE[i] != rhs.ROUTE[i])
      return false;

  if (lhs.DSTS.size() != rhs.DSTS.size())
    return false;

  for (int i = 0; i < lhs.DSTS.size(); i++)
    if (lhs.DSTS[i] != rhs.DSTS[i])
      return false;

  if (lhs.DEMANDS.size() != rhs.DEMANDS.size())
    return false;

  for (int i = 0; i < lhs.DEMANDS.size(); i++)
    if (lhs.DEMANDS[i] != rhs.DEMANDS[i])
      return false;

  // std::cout << lhs.V << " " << rhs.V << std::endl;
  return lhs.ID == rhs.ID && lhs.PRIORITY == rhs.PRIORITY &&
         //    lhs.V == rhs.V &&
         cmp_double(lhs.V, rhs.V) && lhs.TOT_DEMAND == rhs.TOT_DEMAND && lhs.PATH_DISTANCE == rhs.PATH_DISTANCE &&
         lhs.GOOD == rhs.GOOD;
}

bool operator!=(const logistic_sim::Mission &lhs, const logistic_sim::Mission &rhs)
{
  return !(lhs == rhs);
}

ostream &operator<<(ostream &os, const logistic_sim::Path &p)
{
  os << p.PATH.size() << "\n";
  for (auto &v : p.PATH)
  {
    os << v << "\n";
  }
  os << flush;
  return os;
}

ostream &operator<<(ostream &os, const vector<logistic_sim::Path> &v)
{
  os << v.size() << "\n\n";
  for (auto &p : v)
  {
    os << p << "\n";
  }
  os << flush;
  return os;
}

istream &operator>>(istream &is, logistic_sim::Path &p)
{
  p.PATH.clear();
  int n;
  is >> n;
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    p.PATH.push_back(v);
  }

  return is;
}

istream &operator>>(istream &is, vector<logistic_sim::Path> &v)
{
  v.clear();
  int n;
  is >> n;
  for (int i = 0; i < n; i++)
  {
    logistic_sim::Path p;
    is >> p;
    v.push_back(p);
  }

  return is;
}

void TaskPlanner::write_single_mission(ostream &os, const logistic_sim::Mission &m)
{
  os << m.ID << "\n";
  os << m.PRIORITY << "\n";

  os << m.ITEM.size() << "\n";
  for (auto &d : m.ITEM)
  {
    os << d << "\n";
  }

  os << m.ROUTE.size() << "\n";
  for (auto &d : m.ROUTE)
  {
    os << d << "\n";
  }

  os << m.DSTS.size() << "\n";
  for (auto &d : m.DSTS)
  {
    auto it = std::find(dst_vertex.begin(), dst_vertex.end(), d);
    if (it != dst_vertex.end())
    {
      int i = it - dst_vertex.begin();
      os << i << "\n";
    }
  }

  os << m.DEMANDS.size() << "\n";
  for (auto &d : m.DEMANDS)
  {
    os << d << "\n";
  }

  os << m.V << "\n";
  os << m.TOT_DEMAND << "\n";
  os << m.PATH_DISTANCE << "\n";
  os << m.GOOD << "\n";

  os << flush;
}

void TaskPlanner::write_missions(ostream &os, const vector<logistic_sim::Mission> &v)
{
  os << v.size() << "\n\n";
  for (auto &m : v)
  {
    write_single_mission(os, m);
    os << "\n";
  }

  os << flush;
}

logistic_sim::Mission TaskPlanner::read_single_mission(istream &is)
{
  logistic_sim::Mission m;
  int n;

  is >> m.ID;
  is >> m.PRIORITY;

  // is >> m.ITEM.size();
  is >> n;
  m.ITEM.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.ITEM.push_back(v);
  }

  // is >> m.ROUTE.size();
  is >> n;
  m.ROUTE.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.ROUTE.push_back(v);
  }

  // is >> m.DSTS.size();
  is >> n;
  m.DSTS.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.DSTS.push_back(dst_vertex[v]);
  }

  // is >> m.DEMANDS.size();
  is >> n;
  m.DEMANDS.clear();
  for (int i = 0; i < n; i++)
  {
    uint v;
    is >> v;
    m.DEMANDS.push_back(v);
  }

  is >> m.V;
  is >> m.TOT_DEMAND;
  is >> m.PATH_DISTANCE;
  is >> m.GOOD;

  return m;
}

vector<logistic_sim::Mission> TaskPlanner::read_missions(istream &is)
{
  vector<logistic_sim::Mission> v;
  int n;
  is >> n;

  for (int i = 0; i < n; i++)
  {
    logistic_sim::Mission m = read_single_mission(is);
    v.push_back(m);
  }
  return v;
}

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_, const std::string &name) : name(name)
{
  // sub_token = nh_.subscribe("token", 1, &TaskPlanner::token_callback, this);
  // pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

  nh_.setParam("/simulation_running", "true");
}

void TaskPlanner::init(int argc, char **argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::NodeHandle nh;
  read_cmdline_parameters(argc, argv);
  set_map_endpoints(nh);
  calculate_aggregation_paths();
  build_map_graph();  // ONLY CENTRALIZED
  advertise_robot_ready_service(nh);
  advertise_add_missions_service(nh);
  advertise_change_edge_service(nh);
  advertise_remove_vertex_service(nh);
  advertise_add_vertex_service(nh);
  advertise_add_vertex_by_coordinates_service(nh);
  advertise_remove_vertex_by_coordinates_service(nh);
  initialize_amcl_callbacks(nh);
  generate_missions();
  initialize_stats_structure();
  open_times_file();
  sleep(3);
  wait_agents();
  initialize_token();
  detect_offline_mode();

  ROS_INFO("Initialization completed");
}

void TaskPlanner::run()
{
  ROS_INFO_STREAM("Generating mission windows");
  // std::cout << "Generating mission windows" << std::endl;
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
    insert_mission_window(window);
    // window_mutex.lock();
    // mission_windows.push_back(window);
    // ROS_INFO_STREAM("New window aggreagted - to be inserted into token: " << mission_windows.size());
    // ROS_INFO_STREAM("Tasks not yet aggregated: " << missions.size() << "\n");
    // window_mutex.unlock();
  }

  // after computing the last window we wait for shutdown
  ros::waitForShutdown();
}

int TaskPlanner::compute_cost_of_route(std::vector<uint> &route)
{
  int custo_final = 0;
  for (int i = 1; i < route.size(); i++)
  {
    int anterior = route[i - 1];
    int proximo = route[i];

    for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
    {
      if (vertex_web[anterior].id_neigh[j] == proximo)

      {
        custo_final += vertex_web[anterior].cost[j];
        break;
      }
    }
  }
  return custo_final;
}

logistic_sim::Mission TaskPlanner::create_mission(uint type, int id)
{
  static uint d[3] = { 0, 0, 0 };
  logistic_sim::Mission m;
  m.PICKUP = false;
  m.ID = id;
  id++;
  m.PRIORITY = 0;
  m.ITEM.push_back(type);

  copy(std::begin(paths[type]), std::end(paths[type]), back_inserter(m.ROUTE));
  m.DSTS.push_back(dst_vertex[type]);
  m.TOT_DEMAND = (d[type] % 3) + 1;
  m.DEMANDS.push_back((d[type] % 3) + 1);
  d[type]++;
  m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
  m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

  return m;
}

void TaskPlanner::insert_mission_window(std::vector<logistic_sim::Mission> &window)
{
  // mission_windows is accessed also in the token callback thread, hence the mutex
  window_mutex.lock();
  mission_windows.push_back(window);
  ROS_INFO_STREAM("New window aggreagted - to be inserted into token: " << mission_windows.size());
  ROS_INFO_STREAM("Tasks not yet aggregated: " << missions.size() << "\n");
  window_mutex.unlock();
}

int TaskPlanner::my_random(int n)
{
  return rand() % n + 1;
}

void TaskPlanner::random_mission(uint n_missions)
{
  std::vector<logistic_sim::Mission> r;
  uint id_m = 0;
  for (int i = 0; i < n_missions; i++)
  {
    logistic_sim::Mission m;
    auto type = my_random(3);
    m.ID = id_m;
    m.PICKUP = false;
    m.DEMANDS.push_back(my_random(3));
    m.DSTS.push_back(dst_vertex[type - 1]);
    m.TOT_DEMAND = m.DEMANDS.front();
    m.PRIORITY = 0;
    m.ITEM.push_back(type);
    copy(std::begin(paths[type - 1]), std::end(paths[type - 1]), back_inserter(m.ROUTE));
    id_m++;
    m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
    m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    std::cout << m << std::endl;
    missions.push_back(m);
  }
  nTask = missions.size();
}

void TaskPlanner::u_missions_generator()
{
  int size = 6;
  int size_2 = 3;
  int d = 1;

  static int id = 0;
  for (auto i = 0; i < size; i++)
  {
    for (auto j = 0; j < size_2; j++)
    {
      logistic_sim::Mission m;
      m.PICKUP = false;
      m.ID = id;
      id++;
      m.PRIORITY = 0;
      m.ITEM.push_back(i % 3);
      copy(std::begin(paths[i % 3]), std::end(paths[i % 3]), back_inserter(m.ROUTE));
      m.DSTS.push_back(dst_vertex[i % 3]);
      m.TOT_DEMAND = (j % 3) + 1;
      m.DEMANDS.push_back((j % 3) + 1);
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
      missions.push_back(m);

      std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD:" << m.DEMANDS[0]
                << "\n";
    }
  }

  nTask = missions.size();

  for (auto i = 0; i < missions.size(); i++)
  {
    cout << missions[i].TOT_DEMAND << " \n";
  }
}

void TaskPlanner::nu_missions_generator()
{
  uint t1_size = 2;
  uint t2_size = 4;
  uint t3_size = 6;
  static int id = 0;
  while (t1_size > 0 || t2_size > 0 || t3_size > 0)
  {
    if (t1_size > 0)
    {
      logistic_sim::Mission m = create_mission(0, id);
      missions.push_back(m);
      t1_size--;
      id++;
      std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD: " << m.DEMANDS[0]
                << "\tITEM: " << m.ITEM[0] << "\n";
    }
    if (t2_size > 0)
    {
      logistic_sim::Mission m = create_mission(1, id);
      missions.push_back(m);
      t2_size--;
      id++;
      std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD: " << m.DEMANDS[0]
                << "\tITEM: " << m.ITEM[0] << "\n";
    }
    if (t3_size > 0)
    {
      logistic_sim::Mission m = create_mission(2, id);
      missions.push_back(m);
      t3_size--;
      id++;
      std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD: " << m.DEMANDS[0]
                << "\tITEM: " << m.ITEM[0] << "\n";
    }
  }

  nTask = missions.size();

  for (auto i = 0; i < missions.size(); i++)
  {
    cout << missions[i].TOT_DEMAND << " \n";
  }
}

void TaskPlanner::missions_generator(std::string &type_gen)
{
  if (type_gen == "uniform")
  {
    u_missions_generator();
  }
  else if (type_gen == "non-uniform")
  {
    nu_missions_generator();
  }
  else if (type_gen == "file")
  {
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
    missions = read_missions(missions_file);
    nTask = missions.size();
    missions_file.close();
  }
  else if (type_gen == "rand")
  {
    c_print("creating random mission");
    random_mission(12);
  }
  else
  {
    c_print("ERR GEN TYPE", red, P);
  }
}

bool TaskPlanner::robot_ready(logistic_sim::RobotReady::Request &req, logistic_sim::RobotReady::Response &res)
{
  uint id_robot = req.ID_ROBOT;
  ROS_INFO_STREAM("Robot " << id_robot << " is ready");
  // c_print("Robot ", id_robot, " is ready", green, P);
  if (!robots_ready_status[id_robot])
  {
    robots_ready_status[id_robot] = true;
    robots_ready_count++;
  }

  return true;
}

void TaskPlanner::write_missions_on_file(std::string filename)
{
  // scrivo le partizioni generate su file
  boost::filesystem::path results_directory("results");
  if (!boost::filesystem::exists(results_directory))
  {
    boost::filesystem::create_directory(results_directory);
  }

  std::stringstream conf_dir_name;
  conf_dir_name << "missions";
  //<< name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << TEAM_SIZE << "capacity" << TEAM_CAPACITY << "_"
  //<< mapname;
  boost::filesystem::path conf_directory(conf_dir_name.str());
  if (!boost::filesystem::exists(conf_directory))
  {
    boost::filesystem::create_directory(conf_directory);
  }

  // conf_dir_name << "/missions";
  // conf_directory = boost::filesystem::path(conf_dir_name.str());
  // if (!boost::filesystem::exists(conf_directory))
  // {
  //   boost::filesystem::create_directory(conf_directory);
  // }

  if (filename == "")
  {
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
  }

  ofstream missions_file(filename);
  if (missions_file.fail())
  {
    c_print("Can't write missions on disk!!!", red, P);
  }
  else
  {
    //   missions_file << missions;
    write_missions(missions_file, missions);
  }
  missions_file.close();
}

std::vector<logistic_sim::Mission> TaskPlanner::set_partition(const std::vector<logistic_sim::Mission> &ts)
{
  auto start = std::chrono::system_clock::now();
  // c_print("Calculating partitions", green, P);
  ROS_INFO_STREAM("Calculating partitions");
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

  print_coalition(ele);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  times_file << "aggregation:\t" << elapsed_seconds.count() << "\n";
  times_file.flush();
  return ele.first;
}

std::vector<logistic_sim::Path> TaskPlanner::path_partition(logistic_sim::Token &token)
{
  c_print("path_partition on TP Base", red);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

// initialization methods
void TaskPlanner::read_cmdline_parameters(int argc, char **argv)
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
  std::string names = std::string(argv[7]);
  ROS_INFO_STREAM(names);

  initial_kairos_x = std::vector<double>(TEAM_SIZE);
  initial_kairos_y = std::vector<double>(TEAM_SIZE);
  kairos_name = std::vector<std::string>(TEAM_SIZE);
  size_t pos = 0;
  std::string delimiter = ",";
  std::string str_token;
  uint i = 0;
  while ((pos = names.find(delimiter)) != std::string::npos) {
    str_token = names.substr(0, pos);
    kairos_name[i] = str_token;
    ROS_INFO_STREAM(str_token);
    names.erase(0, pos + delimiter.length());
    i++;
  }
  ROS_INFO_STREAM(names);
  kairos_name[i] = names;
}

void TaskPlanner::set_map_endpoints(ros::NodeHandle &nh)
{
  XmlRpc::XmlRpcValue src, dst;
  if (nh.getParam("/src_vertex", src))
  {
    // TODO: single source supported - move to multiple sources!
    ROS_ASSERT(src.getType() == XmlRpc::XmlRpcValue::TypeArray);
    XmlRpc::XmlRpcValue v = src[0];
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeInt);
    src_vertex = (int)v;
    ROS_INFO_STREAM("src_vertex: " << src_vertex);
  }
  else
  {
    ROS_ERROR_STREAM("Can't read param /src_vertex!!!");
    ros::shutdown();
  }

  if (nh.getParam("/dst_vertex", dst))
  {
    ROS_ASSERT(dst.getType() == XmlRpc::XmlRpcValue::TypeArray);
    dst_vertex.clear();
    for (int i = 0; i < dst.size(); i++)
    {
      XmlRpc::XmlRpcValue v = dst[i];
      ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeInt);
      dst_vertex.push_back((int)v);
      ROS_INFO_STREAM("dst_vertex: " << (int)v);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Can't read param /dst_vertex!!!");
    ros::shutdown();
  }

  // find home endpoints from initial_pos parameter
  std::vector<double> list;
  nh.getParam("initial_pos", list);

  if (list.empty())
  {
    list = std::vector<double>(TEAM_SIZE * 2);
    for (int i=0; i<TEAM_SIZE; i++)
    {
      uint robot_id = i;
      std::string robotname = kairos_name[robot_id];
      ROS_WARN("No initial pose given from paremeter, checking KAIROS robot coordinates...");
      ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>(robotname + "/robot_pose", 1, boost::bind(&TaskPlanner::kairos_pose_callback, this, _1, robot_id));
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if (initial_kairos_x[robot_id] != -1 && initial_kairos_y[robot_id] != -1) {
          list[2 * robot_id] = initial_kairos_x[robot_id];
          list[2 * robot_id + 1] = initial_kairos_y[robot_id];
          ROS_INFO_STREAM("Initial pose read from kairos " << robot_id << " coordinates: " << initial_kairos_x[robot_id] << " " << initial_kairos_y[robot_id]);
      } else {
          ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
          ros::shutdown();
          exit(-1);
      }
    }
    reading_initial_kairos_pose = false;
  }

  // if (list.empty())
  // {
  //   ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
  //   ros::shutdown();
  // }
  // read each agent home coordinates to find their home vertex
  for (int value = 0; value < TEAM_SIZE; value++)
  {
    double home_x = list[2 * value];
    double home_y = list[2 * value + 1];
    uint v = IdentifyVertex(vertex_web, dimension, home_x, home_y);
    home_vertex.push_back(v);
    ROS_INFO_STREAM("Robot " << value << " initial vertex: " << v);
  }
  ROS_INFO_STREAM("Map endpoints setted");
}

void TaskPlanner::kairos_pose_callback(const geometry_msgs::Pose::ConstPtr &msg, uint robot_id) {
    if (reading_initial_kairos_pose) {
        initial_kairos_x[robot_id] = msg->position.x;
        initial_kairos_y[robot_id] = msg->position.y;
    }
}

void TaskPlanner::calculate_aggregation_paths()
{
  for (uint dst : dst_vertex)
  {
    int result[100];
    uint result_size;
    dijkstra(src_vertex, dst, result, result_size, vertex_web, dimension);
    paths.push_back(std::vector<uint>(result, result + result_size));
  }
  ROS_INFO_STREAM("Aggregation paths calculated");
}

void TaskPlanner::build_map_graph()
{
}

void TaskPlanner::advertise_robot_ready_service(ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM("Advertising robot_ready service");
  robots_ready_status = std::vector<bool>(TEAM_SIZE);
  robot_ready_service = nh.advertiseService("robot_ready", &TaskPlanner::robot_ready, this);
  if (!robot_ready_service)
  {
    ROS_ERROR_STREAM("Can't create robot_ready service");
  }
  else
  {
    ROS_INFO_STREAM("robot_ready service advertised successfully");
  }
  ros::spinOnce();
  // sleep(30);
}

void TaskPlanner::advertise_add_missions_service(ros::NodeHandle &nh)
{
  ROS_DEBUG_STREAM("Advertising add_missions service");
  add_missions_service = nh.advertiseService("add_missions", &TaskPlanner::add_missions, this);
  if (!add_missions_service)
  {
    ROS_ERROR_STREAM("Can't create add_missions service");
  }
  else
  {
    ROS_INFO_STREAM("add_missions service advertised successfully");
  }
  ros::spinOnce();
}


void TaskPlanner::advertise_remove_vertex_service(ros::NodeHandle &nh)
{

}

void TaskPlanner::advertise_add_vertex_service(ros::NodeHandle &nh)
{

}

void TaskPlanner::advertise_change_edge_service(ros::NodeHandle &nh)
{

}

void TaskPlanner::advertise_add_vertex_by_coordinates_service(ros::NodeHandle &nh)
{

}

void TaskPlanner::advertise_remove_vertex_by_coordinates_service(ros::NodeHandle &nh)
{

}

bool TaskPlanner::add_missions(logistic_sim::AddMissions::Request &msg, logistic_sim::AddMissions::Response &res)
{
  insert_mission_window(msg.MISSION);
  return true;
}

void TaskPlanner::real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot)
{
  last_real_pos[id_robot] = *msg;
}

void TaskPlanner::amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg, int id_robot)
{
  last_amcl_pos[id_robot] = *msg;
  nav_msgs::Odometry *real_pos = &last_real_pos[id_robot];

  double distance = sqrt((msg->pose.pose.position.x - real_pos->pose.pose.position.x) *
                             (msg->pose.pose.position.x - real_pos->pose.pose.position.x) +
                         (msg->pose.pose.position.y - real_pos->pose.pose.position.y) *
                             (msg->pose.pose.position.y - real_pos->pose.pose.position.y));
  // ROS_DEBUG_STREAM("Robot " << id_robot << " position error: " << distance);
  robot_pos_errors[id_robot].push_back(distance);
}

void TaskPlanner::initialize_amcl_callbacks(ros::NodeHandle &nh)
{
  // COMUNE, diventa initialize_amcl_callbacks()
  // subscribe to robot's real and amcl position (to measure error)
  last_real_pos = std::vector<nav_msgs::Odometry>(TEAM_SIZE);
  last_amcl_pos = std::vector<geometry_msgs::PoseWithCovarianceStamped>(TEAM_SIZE);
  robot_pos_errors = std::vector<std::vector<double>>(TEAM_SIZE);
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    std::stringstream real_ss, amcl_ss;
    real_ss << "/robot_" << i << "/base_pose_ground_truth";
    amcl_ss << "/robot_" << i << "/amcl_pose";
    ros::Subscriber real_sub =
        nh.subscribe<nav_msgs::Odometry>(real_ss.str(), 10, boost::bind(&TaskPlanner::real_pos_callback, this, _1, i));
    ros::Subscriber amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        amcl_ss.str(), 10, boost::bind(&TaskPlanner::amcl_pos_callback, this, _1, i));

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
}

void TaskPlanner::write_simple_missions(std::ostream &os, const std::vector<logistic_sim::Mission> &mission)
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

std::vector<logistic_sim::Mission> TaskPlanner::read_simple_missions(std::istream &is)
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

void TaskPlanner::generate_missions()
{
  if (GENERATION == "null")
  {
    nTask = 0;
    return;
  }
  
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
}

void TaskPlanner::initialize_stats_structure()
{
  // initializing stats structure
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    taskplanner::MonitorData data;
    robots_data.push_back(data);
  }
}

void TaskPlanner::open_times_file()
{
  times_file = std::ofstream("times_file.txt", std::ofstream::out | std::ofstream::app);
  times_file << "\n" << mapname << "_" << name << "_" << ALGORITHM << "_" << TEAM_SIZE << "_" << task_set_file << "\n";
}

void TaskPlanner::wait_agents()
{
  while (robots_ready_count < TEAM_SIZE)
  {
    ROS_DEBUG_STREAM("Missing agents: " << TEAM_SIZE - robots_ready_count);
    ros::Duration(1, 0).sleep();
    ros::spinOnce();
  }
}

void TaskPlanner::initialize_token()
{
  logistic_sim::Token token;

  // initialization round
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;
  token.INIT = true;
  token.NEW_MISSIONS_AVAILABLE = false;
  token.SHUTDOWN = false;

  sleep(2);

  pub_token.publish(token);
  ros::spinOnce();

  sleep(1);
}

void TaskPlanner::detect_offline_mode()
{
  // only one window == offline time measurement
  ROS_DEBUG_STREAM("# of items: " << missions.size() << "\twindow size: " << window_size);
  if (missions.size() == window_size)
  {
    ROS_INFO_STREAM("Running in offline mode");
    offline_mode = true;
  }
}

}  // namespace taskplanner

// //------------------------------------------------------------------------------
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "task_planner");

//   ros::NodeHandle nh_;  // con ~ avremmo il prefisso sui topic

//   taskplanner::TaskPlanner TP(nh_);

//   TP.init(argc, argv);

//   c_print("initialization completed!", green);

//   ros::AsyncSpinner spinner(2);

//   spinner.start();

//   ros::waitForShutdown();

//   return 0;
// }
