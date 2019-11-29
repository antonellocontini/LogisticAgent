#pragma once

#include "algorithms.hpp"
#include "boost/filesystem.hpp"

namespace taskplanner
{
ostream &operator<<(ostream &os, const MonitorData &md)
{
  os << md.tot_distance << "," << md.interference_num << "," << md.completed_missions << "," << md.completed_tasks
     << "," << md.total_time;
  return os;
}

ostream &operator<<(ostream &os, const vector<MonitorData> &v)
{
  os << "ID_ROBOT,TOT_DISTANCE,INTERFERENCE_NUM,COMPLETED_MISSIONS,COMPLETED_TASKS,TOTAL_TIME" << endl;
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
  sub_token = nh_.subscribe("token", 1, &TaskPlanner::token_callback, this);
  pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

  nh_.setParam("/simulation_running", "true");
}

void TaskPlanner::init(int argc, char **argv)
{
  srand(time(NULL));
  chdir(PS_path.c_str());
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

  // PARAMETRO TASKSET
  task_set_file = argv[6];

  src_vertex = map_src[mapname];
  dst_vertex = map_dsts[mapname];

  for (uint dst : dst_vertex)
  {
    int result[100];
    uint result_size;
    dijkstra(src_vertex, dst, result, result_size, vertex_web, dimension);
    paths.push_back(std::vector<uint>(result, result + result_size));
  }

  // avvio servizio
  robots_ready_status = std::vector<bool>(TEAM_SIZE);
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("robot_ready", &TaskPlanner::robot_ready, this);
  ros::spinOnce();

  allocate_memory();
  missions_generator(GENERATION);
  // int temp_id = 0;
  // logistic_sim::Mission m;
  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(26);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(33);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(42);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);
  
  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(26);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(33);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(26);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(2);
  // m.DSTS.push_back(33);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(2);
  // m.DSTS.push_back(26);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(33);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // m.ID = temp_id++;
  // m.PICKUP = true;
  // m.DEMANDS.clear();
  // m.DSTS.clear();
  // m.DEMANDS.push_back(1);
  // m.DSTS.push_back(42);
  // m.TOT_DEMAND = m.DEMANDS[0];
  // missions.push_back(m);

  // nTask = 10;

  // print missions
  for (const logistic_sim::Mission &m : missions)
  {
    std::cout << "ID: "  << m.ID << "\n";
    std::cout << "DEMANDS:\n";
    for(auto v : m.DEMANDS)
    {
      std::cout << v << " ";
    }
    std::cout << "DSTS:\n";
    for(auto v : m.DSTS)
    {
      std::cout << v << " ";
    }
    std::cout << std::endl;
  }

  logistic_sim::Token token;

  // GENERAZIONE PARTIZIONI
  // se non leggo da file scrivo su disco le partizioni generate
  if (GENERATION != "file")
  {
    set_partition();
    // scrivo le partizioni generate su file
    boost::filesystem::path results_directory("results");
    if (!boost::filesystem::exists(results_directory))
    {
      boost::filesystem::create_directory(results_directory);
    }

    std::stringstream conf_dir_name;
    conf_dir_name << "results";
                  //<< name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << TEAM_SIZE << "capacity" << TEAM_CAPACITY << "_" << mapname;
    boost::filesystem::path conf_directory(conf_dir_name.str());
    if (!boost::filesystem::exists(conf_directory))
    {
      boost::filesystem::create_directory(conf_directory);
    }

    conf_dir_name << "/missions";
    conf_directory = boost::filesystem::path(conf_dir_name.str());
    if (!boost::filesystem::exists(conf_directory))
    {
      boost::filesystem::create_directory(conf_directory);
    }

    int run_number = 1;
    std::stringstream filename;
    std::ifstream check_new;
    // loop per controllare se il file già esiste
    do
    {
      filename.str("");  // cancella la stringa
      filename << conf_dir_name.str() << "/" << run_number << ".txt";
      check_new = std::ifstream(filename.str());
      run_number++;
    } while (check_new);
    check_new.close();

    ofstream missions_file(filename.str());
    if (missions_file.fail())
    {
      c_print("Impossibile scrivere missioni su disco!!!", red, P);
    }
    else
    {
      //   missions_file << missions;
      write_missions(missions_file, missions);
    }
    missions_file.close();
  }

  // GENERAZIONE PERCORSI
  static std::vector<logistic_sim::Path> paths(TEAM_SIZE, logistic_sim::Path());
  // if (GENERATION != "file")
  if (true)
  {
    paths = path_partition(token);
  }
  else if (name == "GlobalTaskPlanner" ||
           name == "GreedyTaskPlanner")  // se leggo da file e uso algoritmo global leggo i percorsi da disco
  {
    std::string filename("paths_file.txt");
    boost::filesystem::path paths_path(filename);
    if (!boost::filesystem::exists(paths_path))
    {
      c_print("File percorsi ", filename, " non esistente!!!", red, P);
      sleep(1);
      ros::shutdown();
      system("./stop_experiment.sh");
    }

    ifstream paths_file(filename);
    paths_file >> paths;
    if (paths.size() != TEAM_SIZE)
    {
      c_print("Numero robot incoerente!!! TEAMSIZE: ", TEAM_SIZE, " file: ", paths.size(), red, P);
    }
    paths_file.close();

    // prendo statistiche per token
    filename = "stats_file.txt";
    boost::filesystem::path stats_path(filename);
    if (!boost::filesystem::exists(stats_path))
    {
      c_print("File statistiche ", filename, " non esistente!!!", red, P);
      sleep(1);
      ros::shutdown();
      system("./stop_experiment.sh");
    }

    ifstream stats_file(filename);
    int n;
    stats_file >> n;
    if (n != TEAM_SIZE)
    {
      c_print("Numero robot incoerente!!! TEAMSIZE: ", TEAM_SIZE, " file: ", n, red, P);
    }
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      uint m, t;
      stats_file >> m;
      stats_file >> t;
      token.MISSIONS_COMPLETED.push_back(m);
      token.TASKS_COMPLETED.push_back(t);
    }
    stats_file.close();
  }
  // for(int i=0; i<TEAM_SIZE; i++)
  // {
  //     std::cout << "Robot " << i << " path:\n";
  //     for (uint v : paths[i].PATH)
  //     {
  //         std::cout << setw(2) << v << " ";
  //     }
  //     std::cout << "\n\n";
  // }
  // std::cout << std::endl;

  c_print("TEAM: ", TEAM_SIZE, " nTask: ", nTask, magenta);

  // aspetto che arrivino gli agenti
  while (robots_ready_count < TEAM_SIZE)
  {
    ros::Duration(1, 0).sleep();
    ros::spinOnce();
  }
  // sleep(10);

  // giro di inizializzazione
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;
  token.INIT = true;
  token.GOOD_PATHS = true;
  token.CALCULATE_PATHS = false;
  // ros bool diventa uint8_t nei messaggi
  token.TAKEN_MISSION = std::vector<int>(missions.size(), -1);
  // se il task planner ha calcolato i percorsi li metto nel token
  if (name == "GlobalTaskPlanner" || name == "GreedyTaskPlanner")
  {
    token.TRAILS = paths;
    // se ho generato i percorsi li scrivo su disco
    if (GENERATION != "file")
    {
      std::string filename("paths_file.txt");
      ofstream paths_file(filename);
      if (!paths_file.fail())
      {
        paths_file << paths;
      }
      else
      {
        c_print("Impossibile scrivere percorsi su disco!!!", red, P);
      }
      paths_file.close();
    }
  }

  pub_token.publish(token);
  ros::spinOnce();

  sleep(1);

  c_print("INIT", green);
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
  int size = 3;
  int size_2 = 3;
  int d = 1;

  static int id = 0;
  for (auto i = 0; i < size; i++)
  {
    // c_print("oh");
    for (auto j = 0; j < size_2; j++)
    {
      // c_print("eh");
      logistic_sim::Mission m;
      m.PICKUP = false;
      m.ID = id;
      id++;
      m.PRIORITY = 0;
      m.ITEM.push_back(i % 3);
      // c_print("ah");
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
    // taskset_dir_name << "results/" << name << "_" << ALGORITHM
    //                  // TODO: lettura da cartella diversa da rand!!!
    //                  << "_"
    //                  << "rand"
    //                  << "_teamsize" << TEAM_SIZE << "capacity" << TEAM_CAPACITY << "_" << mapname << "/missions/"
    //                  << task_set_file;
    taskset_dir_name << "results/missions/" << task_set_file;
    std::string filename(taskset_dir_name.str());
    boost::filesystem::path missions_path(filename);
    if (!boost::filesystem::exists(missions_path))
    {
      c_print("File missioni ", filename, " non esistente!!!", red, P);
      sleep(1);
      ros::shutdown();
      system("./stop_experiment.sh");
    }
    ifstream missions_file(filename);
    // missions_file >> missions;
    missions = read_missions(missions_file);
    missions_file.close();
  }
  else if (type_gen == "rand")
  {
    c_print("creating random mission");
    random_mission(10);
  }
  else
  {
    c_print("ERR GEN TYPE", red, P);
  }
}

void TaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{
  static int last_mission_size = 0;
  static vector<int> last_interf_count;
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
    for (auto m : missions)
    {
      for (auto dst : m.DSTS)
      {
        std::cout << dst << " ";
      }
      std::cout << "\n";
    }
    std::cout << "\n";
    token.MISSION = missions;
    token.END_SIMULATION = false;
    start_time = ros::Time::now();
    last_mission_size = missions.size();
  }
  else
  {
    token.HEADER.seq += 1;
    // calcolo quanti robot ci sono e preparo la struttura dati
    if (first_round)
    {
      // per come i robot sono ordinati l'ID del sender equivale al numero di robot meno 1
      num_robots = msg->ID_SENDER + 1;
      for (int i = 0; i < num_robots; i++)
      {
        MonitorData data;
        robots_data.push_back(data);
      }

      last_interf_count = vector<int>(num_robots, 0);

      first_round = false;
    }

    // stampa task rimanenti
    if (token.MISSION.size() < last_mission_size)
    {
      last_mission_size = token.MISSION.size();
      c_print("Task rimanenti: ", last_mission_size, green);
    }

    // stampa quando avviene interferenza
    for (int i = 0; i < last_interf_count.size(); i++)
    {
      if (token.INTERFERENCE_COUNTER[i] > last_interf_count[i])
      {
        last_interf_count[i] = token.INTERFERENCE_COUNTER[i];
        c_print("Interferenza rilevata dal robot ", i, "!", red);
      }
    }

    // aggiorno la mia struttura con i dati del token
    for (int i = 0; i < num_robots; i++)
    {
      robots_data[i].interference_num = token.INTERFERENCE_COUNTER[i];
      robots_data[i].completed_missions = token.MISSIONS_COMPLETED[i];
      robots_data[i].completed_tasks = token.TASKS_COMPLETED[i];
      robots_data[i].tot_distance = token.TOTAL_DISTANCE[i];
      robots_data[i].total_time = (ros::Time::now() - start_time).sec;
    }

    if (token.INIT_POS.empty())
    {
      boost::filesystem::path results_directory("results");
      if (!boost::filesystem::exists(results_directory))
      {
        boost::filesystem::create_directory(results_directory);
      }

      std::stringstream conf_dir_name;
      conf_dir_name << "results/" << name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << num_robots
                    << "capacity" << CAPACITY[0] << "_" << mapname;
      boost::filesystem::path conf_directory(conf_dir_name.str());
      if (!boost::filesystem::exists(conf_directory))
      {
        boost::filesystem::create_directory(conf_directory);
      }

      int run_number = 1;
      std::stringstream filename;
      std::ifstream check_new;
      // loop per controllare se il file già esiste
      // do
      // {
      //   filename.str("");  // cancella la stringa
      //   filename << conf_dir_name.str() << "/" << run_number << ".csv";
      //   check_new = std::ifstream(filename.str());
      //   run_number++;
      // } while (check_new);
      // check_new.close();

      filename << conf_dir_name.str() << "/" << task_set_file << ".csv";

      ofstream stats(filename.str());
      stats << robots_data;
      stats.close();
      ros::NodeHandle nh;
      nh.setParam("/simulation_running", "false");
      token.END_SIMULATION = true;
    }
  }

  pub_token.publish(token);
  ros::spinOnce();
  if (token.END_SIMULATION)
  {
    ros::shutdown();
    system("./stop_experiment.sh");
  }
}

bool TaskPlanner::robot_ready(logistic_sim::RobotReady::Request &req, logistic_sim::RobotReady::Response &res)
{
  uint id_robot = req.ID_ROBOT;
  c_print("Robot ", id_robot, " is ready", green, P);
  if (!robots_ready_status[id_robot])
  {
    robots_ready_status[id_robot] = true;
    robots_ready_count++;
  }

  return true;
}

}  // namespace taskplanner
