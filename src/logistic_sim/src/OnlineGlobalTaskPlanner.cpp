#include "OnlineGlobalTaskPlanner.hpp"
#include "permutations.hpp"

namespace onlineglobaltaskplanner
{
OnlineGlobalTaskPlanner::OnlineGlobalTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
{
  sub_token = nh_.subscribe("token", 1, &OnlineGlobalTaskPlanner::token_callback, this);
  pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

  nh_.setParam("/simulation_running", "true");
}

void OnlineGlobalTaskPlanner::init(int argc, char **argv)
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

  // PARAMETRO TASKSET
  task_set_file = argv[6];

  src_vertex = map_src[mapname];
  dst_vertex = map_dsts[mapname];

  // costruisce percorsi per euristica
  for (uint dst : dst_vertex)
  {
    int result[100];
    uint result_size;
    dijkstra(src_vertex, dst, result, result_size, vertex_web, dimension);
    paths.push_back(std::vector<uint>(result, result + result_size));
  }

  // alloco memoria per astar
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

  // avvio servizio
  robots_ready_status = std::vector<bool>(TEAM_SIZE);
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("robot_ready", &OnlineGlobalTaskPlanner::robot_ready, this);
  ros::spinOnce();

  // subscribe alle posizioni dei robot
  last_real_pos = std::vector<nav_msgs::Odometry>(TEAM_SIZE);
  last_amcl_pos = std::vector<geometry_msgs::PoseWithCovarianceStamped>(TEAM_SIZE);
  robot_pos_errors = std::vector<std::vector<double>>(TEAM_SIZE);
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    std::stringstream real_ss, amcl_ss;
    real_ss << "/robot_" << i << "/base_pose_ground_truth";
    amcl_ss << "/robot_" << i << "/amcl_pose";
    ros::Subscriber real_sub = nh.subscribe<nav_msgs::Odometry>(
        real_ss.str(), 10, boost::bind(&OnlineGlobalTaskPlanner::real_pos_callback, this, _1, i));
    ros::Subscriber amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        amcl_ss.str(), 10, boost::bind(&OnlineGlobalTaskPlanner::amcl_pos_callback, this, _1, i));

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
    c_print("Lettura da file...", green, P);
    std::stringstream taskset_dir_name;
    taskset_dir_name << "missions/" << task_set_file;
    std::string filename(taskset_dir_name.str());
    boost::filesystem::path missions_path(filename);
    if (!boost::filesystem::exists(missions_path))
    {
      c_print("File missioni ", filename, " non esistente!!!", red, P);
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

  c_print("Numero di task: ", missions.size(), green, P);
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

  // inizializzo strutture statistiche
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    taskplanner::MonitorData data;
    robots_data.push_back(data);
  }

  // aspetto che arrivino gli agenti
  while (robots_ready_count < TEAM_SIZE)
  {
    ros::Duration(1, 0).sleep();
    ros::spinOnce();
  }

  // wait
  ros::Duration(3.0).sleep();

  logistic_sim::Token token;

  // giro di inizializzazione
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

void OnlineGlobalTaskPlanner::run()
{
  std::cout << "Genero finestre di task" << std::endl;
  while (!missions.empty())
  {
    /*
     * in questo loop creo le finestre di task multi-item
     * da inserire nel token, ogni task dovrebbe indicare
     * a quale finestra appartiene in modo da non intervallarli
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
    // il mutex è necessario perchè nella thread del token
    // si accede al vettore delle finestre
    window_mutex.lock();
    mission_windows.push_back(window);
    c_print("Nuova finestra calcolata - Finestre rimanenti: ", mission_windows.size(), yellow, P);
    c_print("Task ancora da unire: ", missions.size(), green, P);
    c_print("");
    window_mutex.unlock();
  }

  // una volta calcolata l'ultima finestra si attende la terminazione
  ros::waitForShutdown();
}

/*
 * qua gestisco la raccolta delle statistiche (come sempre)
 * inoltre devo gestire l'inserimento di nuovi task multi-item
 * quando sono pronti (necessario meccanismo di sincronizzazione)
 *
 * una volta inseriti i nuovi task nel token si abilita una flag
 * che indica agli agenti che nuovi task sono stati inseriti e che
 * devono allocare e pianificare
 *
 * per ora facciamo tornare gli agenti a casa alla fine di ogni
 * finestra quindi non ci dovrebbero essere problemi con la pianificazione
 *
 * deve sempre valere il solito meccanismo di rimozione robot in
 * caso di fallimento
 *
 * con una nuova finestra i robot spenti devono essere
 * riabilitati
 */
void OnlineGlobalTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
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

    // stampa task rimanenti
    // if (token.MISSION.size() != last_mission_size)
    // {
    //     last_mission_size = token.MISSION.size();
    //     c_print("Task rimanenti: ", last_mission_size, green);
    // }

    // se c'è una nuova finestra pronta la aggiungo al token
    window_mutex.lock();
    // la flag viene messa a false dagli agenti quando
    // hanno finito di distribuirsi le nuove missioni
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
      // se non ci sono più missioni da inserire si indica nel token
      if (missions.empty())
      {
        token.ALL_MISSIONS_INSERTED = true;
      }
      c_print("Finestra inserita nel token - Finestre rimanenti: ", mission_windows.size(), yellow, P);
      token.SYNCED_ROBOTS = false;
      token.NEW_MISSIONS_AVAILABLE = false;
    }
    window_mutex.unlock();

    // controllo liveness dei robot
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

    // controllo conflitti nei percorsi
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

    // aggiorno la mia struttura con i dati del token
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      robots_data[i].interference_num = token.INTERFERENCE_COUNTER[i];
      robots_data[i].completed_missions = token.MISSIONS_COMPLETED[i];
      robots_data[i].completed_tasks = token.TASKS_COMPLETED[i];
      robots_data[i].tot_distance = token.TOTAL_DISTANCE[i];
      robots_data[i].total_time = (ros::Time::now() - start_time).sec;
    }

    // all'uscita scrivo le statistiche su disco
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
        // loop per controllare se il file già esiste
        do
        {
          filename.str("");  // cancella la stringa
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
      // esplicitare il namespace
      // per usare l'operatore corretto
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

std::vector<logistic_sim::Path> OnlineGlobalTaskPlanner::path_partition(logistic_sim::Token &token,
                                                                        std::vector<logistic_sim::Mission> &missions)
{
  ofstream stats_file("stats_file.txt");
  c_print("Calculating tasks distribution", green, P);
  // unsico le due parti di percorso di tutti i robot
  std::vector<logistic_sim::Path> robot_paths;
  std::vector<std::vector<logistic_sim::Path>> best_paths(TEAM_SIZE);
  std::vector<uint> best_paths_length(TEAM_SIZE, std::numeric_limits<uint>::max());
  std::vector<bool> possible_paths_found(TEAM_SIZE, false);
  std::vector<logistic_sim::Token> missions_stats(TEAM_SIZE);
  logistic_sim::Token best_token;

  // individuo vertici iniziali degli agenti
  std::vector<uint> init_vertex;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    init_vertex.push_back(token.TRAILS[i].PATH.back());
  }

  // individuo vertici casa degli agenti
  std::vector<uint> home_vertex;
  ros::NodeHandle nh;
  std::vector<double> list;
  nh.getParam("initial_pos", list);

  if (list.empty())
  {
    ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
    ros::shutdown();
    exit(-1);
  }

  // dalle coordinate degli agenti individuo il nodo del grafo
  for (int value = 0; value < TEAM_SIZE; value++)
  {
    double initial_x = list[2 * value];
    double initial_y = list[2 * value + 1];
    uint v = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    home_vertex.push_back(v);
    // std::cout << "Robot " << value << "\tVertice iniziale: " << v << std::endl;
  }

  for (int current_team_size = TEAM_SIZE; current_team_size >= 1; current_team_size--)
  {
    try
    {
      c_print("Missions number: ", missions.size(), green, P);
      partition::iterator it(missions.size());
      int id_partition = 0;

      std::vector<uint> waypoints;
      logistic_sim::Token temp_token;
      temp_token.MISSIONS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      temp_token.TASKS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      while (true)
      {
        std::vector<std::vector<logistic_sim::Mission>> partitions = *it[missions];
        auto n_subsets = it.subsets();
        // if (n_subsets <= TEAM_SIZE)
        if (n_subsets == current_team_size)
        {
          // std::cout << "partition for " << n_subsets << " robots" << std::endl;
          // per i robot fermi
          if (n_subsets < TEAM_SIZE)
          {
            partitions.insert(partitions.end(), TEAM_SIZE - n_subsets, std::vector<logistic_sim::Mission>());
          }
          perm_iterator<std::vector<logistic_sim::Mission>> jt(partitions);
          try
          {
            while (true)
            {
              std::vector<std::vector<logistic_sim::Mission>> permutation = *jt;

              // assegno la permutazione corrente ai robot e provo a calcolare i percorsi
              temp_token.TRAILS = token.TRAILS;
              temp_token.HOME_TRAILS = token.HOME_TRAILS;
              robot_paths = temp_token.TRAILS;
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                robot_paths[i].PATH.insert(robot_paths[i].PATH.end(), temp_token.HOME_TRAILS[i].PATH.begin(),
                                           temp_token.HOME_TRAILS[i].PATH.end());
              }

              for (int i = 0; i < TEAM_SIZE; i++)
              {
                temp_token.MISSIONS_COMPLETED[i % TEAM_SIZE] = token.MISSIONS_COMPLETED[i % TEAM_SIZE];
                temp_token.TASKS_COMPLETED[i % TEAM_SIZE] = token.TASKS_COMPLETED[i % TEAM_SIZE];
                if (!permutation[i].empty())
                {
                  // std::cout << "Robot " << i << " WAYPOINTS\n";
                  waypoints.clear();
                  waypoints.push_back(init_vertex[i % TEAM_SIZE]);
                  for (logistic_sim::Mission &m : permutation[i])
                  {
                    temp_token.MISSIONS_COMPLETED[i % TEAM_SIZE]++;
                    for (uint d : m.DEMANDS)
                    {
                      temp_token.TASKS_COMPLETED[i % TEAM_SIZE]++;
                    }
                    waypoints.push_back(src_vertex);
                    for (uint v : m.DSTS)
                    {
                      waypoints.push_back(v);
                    }
                  }
                  waypoints.push_back(home_vertex[i % TEAM_SIZE]);

                  for (auto &w : waypoints)
                  {
                    // std::cout << w << " ";
                  }
                  // std::cout << "\n" << std::endl;
                  try
                  {
                    std::vector<unsigned int> last_leg, first_leg;
                    auto f = boost::bind(astar_cmp_function, min_hops_matrix, waypoints, _1, _2);
                    std::vector<uint> path =
                        spacetime_dijkstra(robot_paths, map_graph, waypoints, i % TEAM_SIZE,
                                           temp_token.TRAILS[i % TEAM_SIZE].PATH.size() - 1, &last_leg, &first_leg, &f);

                    temp_token.TRAILS[i % TEAM_SIZE].PATH.pop_back();
                    temp_token.TRAILS[i % TEAM_SIZE].PATH.insert(temp_token.TRAILS[i % TEAM_SIZE].PATH.end(),
                                                                 first_leg.begin(), first_leg.end());
                    temp_token.HOME_TRAILS[i % TEAM_SIZE].PATH = last_leg;

                    robot_paths[i % TEAM_SIZE].PATH = temp_token.TRAILS[i % TEAM_SIZE].PATH;
                    robot_paths[i % TEAM_SIZE].PATH.insert(robot_paths[i % TEAM_SIZE].PATH.end(),
                                                           temp_token.HOME_TRAILS[i % TEAM_SIZE].PATH.begin(),
                                                           temp_token.HOME_TRAILS[i % TEAM_SIZE].PATH.end());
                  }
                  catch (std::string &e)
                  {
                    // empty path means planning has failed
                    robot_paths[i % TEAM_SIZE].PATH.clear();
                  }
                }
              }
              // std::cout << std::endl;

              // check for empty path
              int empty_paths = 0;
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                if (robot_paths[i].PATH.empty())
                {
                  empty_paths++;
                }
              }

              if (empty_paths == 0)
              {
                // controllo se i percorsi che ho trovato sono
                // più corti dei migliori trovati finora
                uint max_length = robot_paths[0].PATH.size();
                for (int i = 1; i < TEAM_SIZE; i++)
                {
                  if (robot_paths[i].PATH.size() > max_length)
                  {
                    max_length = robot_paths[i].PATH.size();
                  }
                }

                if (max_length < best_paths_length[n_subsets - 1])
                {
                  c_print("new best with ", n_subsets, " robots: ", max_length, green, P);
                  best_paths_length[n_subsets - 1] = max_length;
                  best_paths[n_subsets - 1] = robot_paths;
                  possible_paths_found[n_subsets - 1] = true;
                  missions_stats[n_subsets - 1] = temp_token;
                  best_token = temp_token;
                }
              }
              ++jt;
            }
          }
          catch (std::overflow_error &e)
          {
          }
        }
        ++it;
      }
    }
    catch (std::overflow_error &)
    {
    }

    if (possible_paths_found[current_team_size - 1])
    {
      break;
    }
  }

  // se sono riuscito a trovare almeno un insieme
  // di percorsi validi lo restituisco
  for (int j = TEAM_SIZE - 1; j >= 0; j--)
  {
    if (possible_paths_found[j])
    {
      std::cout << "Trovato!" << std::endl;
      //   stats_file << TEAM_SIZE << "\n\n";
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        token.MISSIONS_COMPLETED = best_token.MISSIONS_COMPLETED;
        token.TASKS_COMPLETED = best_token.TASKS_COMPLETED;
        token.TRAILS = best_token.TRAILS;
        token.HOME_TRAILS = best_token.HOME_TRAILS;
        // stats_file << missions_stats[j].MISSIONS_COMPLETED[i] << "\n";
        // stats_file << missions_stats[j].TASKS_COMPLETED[i] << "\n\n";
        std::cout << "Percorso robot " << i << "\n";
        for (uint v : best_paths[j][i].PATH)
        {
          std::cout << v << " ";
        }
        std::cout << "\n" << std::endl;
      }
      stats_file.close();
      return best_paths[j];
    }
  }

  c_print("[ERROR]Can't find solution!!!", red, P);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

std::vector<logistic_sim::Mission> OnlineGlobalTaskPlanner::set_partition(const std::vector<logistic_sim::Mission> &ts)
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

        // rimuovo doppioni adiacenti in DSTS
        for (int j = 0; j < candidate_subset.DSTS.size() - 1; j++)
        {
          if (candidate_subset.DSTS[j] == candidate_subset.DSTS[j + 1])
          {
            candidate_subset.DSTS.erase(candidate_subset.DSTS.begin() + j + 1);
            j--;
          }
        }

        // calcolo la lunghezza del percorso di questa missione
        // per poter stimare la metrica V
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
        // uso PICKUP == true se non è buona
        // prima soglio e controllo che la partizione sia composta da sottoinsiemi processabili dai robot poi calcolo il
        // percorso e metto in ordine per V poi prendo la prima buona
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
 * Floyd-Warshall per trovare il minimo numero di hop tra ogni coppia di vertici
 */
std::vector<std::vector<unsigned int>> OnlineGlobalTaskPlanner::calculate_min_hops_matrix()
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

bool OnlineGlobalTaskPlanner::check_paths_conflicts(const std::vector<logistic_sim::Path> &paths, bool print)
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
          ss << "[WARN] Robot " << ri << " e " << rj << " si incontrano in " << p1.PATH[i] << " all'istante " << i;
          c_print(ss.str(), yellow, print);
        }
        if (p1.PATH[i] == p2.PATH[i + 1] && p2.PATH[i] == p1.PATH[i + 1])
        {
          conflicts++;
          good = false;
          std::stringstream ss;
          ss << "[WARN] Robot " << ri << " e " << rj << "si incontrano nell'arco (" << p1.PATH[i] << ","
             << p1.PATH[i + 1] << ") all'istante " << i;
          c_print(ss.str(), yellow, print);
        }
      }
    }
  }

  if (conflicts > 0)
  {
    c_print("[WARN] individuati ", conflicts, "conflitti", yellow, print);
  }

  return good;
}

void OnlineGlobalTaskPlanner::write_simple_missions(std::ostream &os,
                                                    const std::vector<logistic_sim::Mission> &missions)
{
  os << missions.size() << "\n\n";
  for (const logistic_sim::Mission &m : missions)
  {
    os << m.ID << "\n";
    uint dst = m.DSTS[0];
    // cerco indice nel vettore di destinazioni
    auto it = std::find(dst_vertex.begin(), dst_vertex.end(), dst);
    os << it - dst_vertex.begin() << "\n";

    uint dms = m.DEMANDS[0];
    os << dms << "\n";

    os << "\n";
  }

  os << std::flush;
}

std::vector<logistic_sim::Mission> OnlineGlobalTaskPlanner::read_simple_missions(std::istream &is)
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

    // genero percorso e metrica V
    copy(paths[dst_index].begin(), paths[dst_index].end(), back_inserter(m.ROUTE));
    m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
    m.TOT_DEMAND = dms;
    m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

    missions.push_back(m);
  }

  return missions;
}

void OnlineGlobalTaskPlanner::real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot)
{
  last_real_pos[id_robot] = *msg;
}

void OnlineGlobalTaskPlanner::amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg,
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

}  // namespace onlineglobaltaskplanner

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlineglobaltaskplanner::OnlineGlobalTaskPlanner OGTP(nh_);
  OGTP.init(argc, argv);
  c_print("inizializzazione finita!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  OGTP.run();
  return 0;
}