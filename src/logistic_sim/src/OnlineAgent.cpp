#include <limits>
#include "OnlineAgent.hpp"

namespace onlineagent
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

OnlineAgent::~OnlineAgent()
{
  for (unsigned int i = 0; i < dimension; i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_TIME; k++)
      {
        delete[] prev_paths[i][k][j];
      }
      delete[] prev_paths[i][j];
      delete[] path_sizes[i][j];
      delete[] visited[i][j];
    }

    delete[] prev_paths[i];
    delete[] path_sizes[i];
    delete[] visited[i];
  }
  delete[] prev_paths;
  delete[] path_sizes;
  delete[] visited;
  delete[] queue;
}

void OnlineAgent::allocate_memory()
{
  std::cout << "allocating memory..." << std::endl;
  std::cout << dimension << " " << MAX_TIME << std::endl;
  prev_paths = new unsigned int ***[dimension];
  path_sizes = new unsigned int **[dimension];
  for (unsigned int i = 0; i < dimension; i++)
  {
    prev_paths[i] = new unsigned int **[MAX_TIME];
    path_sizes[i] = new unsigned int *[MAX_TIME];
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      prev_paths[i][j] = new unsigned int *[MAX_WAYPOINTS];
      path_sizes[i][j] = new unsigned int[MAX_WAYPOINTS];
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        prev_paths[i][j][k] = new unsigned int[MAX_TIME];
      }
    }
  }

  visited = new unsigned int **[dimension];
  for (unsigned int i = 0; i < dimension; i++)
  {
    visited[i] = new unsigned int *[MAX_TIME];
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      visited[i][j] = new unsigned int[MAX_WAYPOINTS];
    }
  }

  queue = new st_location[MAX_TIME * MAX_TIME];
}


void OnlineAgent::init(int argc, char **argv)
{
  Agent::init(argc, argv);
  allocate_memory();
  map_graph = build_graph();
  min_hops_matrix = calculate_min_hops_matrix();
  // TEST
  // unsigned int infinity = std::numeric_limits<unsigned int>::max();
  // std::cout << "    | ";
  // for(int i=0; i<dimension; i++)
  // {
  //   std::cout << std::setw(3) << i << " ";
  // }
  // std::cout << "\n----+-";
  // for(int i=0; i<dimension; i++)
  // {
  //   std::cout << "----";
  // }
  // std::cout << "\n";
  // for(int i=0; i<dimension; i++)
  // {
  //   std::cout << std::setw(3) << i << " | ";
  //   for(int j=0; j<dimension; j++)
  //   {
  //     if (min_hops_matrix[i][j] == infinity)
  //       std::cout << "inf ";
  //     else
  //       std::cout << std::setw(3) << min_hops_matrix[i][j] << " ";
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << std::endl;
}

void OnlineAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
  // se non è per me termino
  if (msg->ID_RECEIVER != ID_ROBOT)
    return;

  logistic_sim::Token token;
  token = *msg;

  token.ID_SENDER = ID_ROBOT;
  if (msg->ID_RECEIVER == TEAM_SIZE - 1)
  {
    token.ID_RECEIVER = TASK_PLANNER_ID;
  }
  else
  {
    token.ID_RECEIVER = (ID_ROBOT + 1) % TEAM_SIZE;
  }

  if (msg->INIT)
  {
    c_print("Initialization...", green, P);
    token.CAPACITY.push_back(CAPACITY);
    token.INIT_POS.push_back(initial_vertex);
    token.CURR_VERTEX.push_back(initial_vertex);
    token.NEXT_VERTEX.push_back(initial_vertex);
    token.MISSION_START_TIME.push_back(ros::Time::now());
    token.MISSION_CURRENT_DISTANCE.push_back(0.0f);
    token.INTERFERENCE_COUNTER.push_back(0);
    token.MISSIONS_COMPLETED.push_back(0);
    token.TASKS_COMPLETED.push_back(0);
    token.TOTAL_DISTANCE.push_back(0.0f);
    token.X_POS.push_back(0.0);
    token.Y_POS.push_back(0.0);
    token.GOAL_STATUS.push_back(0);
    logistic_sim::Path p;
    p.PATH = std::vector<uint>(1, initial_vertex);
    token.TRAILS.push_back(p);
    token.NEW_TRAILS.push_back(p);
    token.HOME_TRAILS.push_back(logistic_sim::Path());
    token.REACHED_HOME.push_back(false);
    token.FAILED_ALLOCATION.push_back(false);
    token.ACTIVE_ROBOTS = TEAM_SIZE;

    initialize = false;
    next_vertex = current_vertex = initial_vertex;
    // goal_complete = true;
    // sendGoal(initial_vertex);
    goal_success = true;
  }
  else if (msg->ALLOCATE)
  {
    token_priority_alloc_plan(msg, token);
  }
  else
  {
    // avanzamento dei robot
    token_priority_coordination(msg, token);
    // token_simple_coordination(msg, token);
  }
  // VECCHIO METODO
  // if (msg->CALCULATE_PATHS)
  // {
  //   token_simple_planning(msg, token);
  // }
  // else if (msg->ALLOCATE)
  // {
  //   token_simple_allocation(msg, token);
  // }

  ros::Duration(0.03).sleep();
  token_pub.publish(token);
  ros::spinOnce();

  if (token.SHUTDOWN)
  {
    c_print("Segnale di shutdown, termino...", magenta, P);
    ros::shutdown();
  }
}

void OnlineAgent::token_simple_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  // avanzamento dei robot

  // aggiorno posizioni dei robot nel token
  token.X_POS[ID_ROBOT] = xPos[ID_ROBOT];
  token.Y_POS[ID_ROBOT] = yPos[ID_ROBOT];
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (i != ID_ROBOT)
    {
      xPos[i] = token.X_POS[i];
      yPos[i] = token.Y_POS[i];
    }
  }

  // gestisco l'arrivo al goal
  if (goal_complete)
  {
    static bool first_time = true;

    if (first_time)
    {
      // aggiorno distanza percorsa nel token
      uint edge_length = 0;
      if (current_vertex < dimension)
      {
        for (int i = 0; i < vertex_web[current_vertex].num_neigh; i++)
        {
          if (vertex_web[current_vertex].id_neigh[i] == next_vertex)
          {
            edge_length += vertex_web[current_vertex].cost[i];
            break;
          }
        }
      }
      token.TOTAL_DISTANCE[ID_ROBOT] += edge_length;

      if (token.GOAL_STATUS[ID_ROBOT] > 0 && token.TRAILS[ID_ROBOT].PATH.size() > 1)
      {
        token.TRAILS[ID_ROBOT].PATH.erase(token.TRAILS[ID_ROBOT].PATH.begin());
      }
      token.GOAL_STATUS[ID_ROBOT]++;

      // TODO: bisogna gestire condizione di terminazione
      if (!token.NEW_MISSIONS_AVAILABLE && token.ALL_MISSIONS_INSERTED && token.TRAILS[ID_ROBOT].PATH.size() == 1)
      {
        token.REACHED_HOME[ID_ROBOT] = true;
        bool can_exit = true;
        for (int i = 0; i < TEAM_SIZE; i++)
        {
          if (!token.REACHED_HOME[i])
          {
            can_exit = false;
            break;
          }
        }

        if (can_exit)
        {
          c_print("Finiti tutti i task, esco!", magenta, P);
          token.SHUTDOWN = true;
        }
      }

      current_vertex = next_vertex;
      first_time = false;
    }

    bool equal_status = true;
    int status = token.GOAL_STATUS[0];
    for (int i = 1; i < TEAM_SIZE; i++)
    {
      if (token.GOAL_STATUS[i] != status)
        equal_status = false;
    }

    if (equal_status)
    {
      // se ci sono nuove missioni disponibili bisogna allocarle
      // IMPORTANTE: è essenziale che i robot siano sincronizzati
      // sul goal altrimenti la pianificazione si baserà
      // su dei percorsi sfasati
      if (ID_ROBOT == TEAM_SIZE - 1 && msg->NEW_MISSIONS_AVAILABLE)
      {
        token.ALLOCATE = true;
      }

      first_time = true;
      if (token.TRAILS[ID_ROBOT].PATH.size() > 1)
      {
        goal_complete = false;
        next_vertex = token.TRAILS[ID_ROBOT].PATH[1];
      }
      else
      {
        c_print("[ WARN]Don't know what to do!!!", yellow, P);
        next_vertex = current_vertex;
      }
      c_print("before OnGoal()", magenta);
      c_print("[DEBUG]\tGoing to ", next_vertex, green, P);
      sendGoal(next_vertex);
    }
  }

  // metto nel token quale arco sto occupando
  token.CURR_VERTEX[ID_ROBOT] = current_vertex;
  token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
}

void OnlineAgent::token_simple_allocation(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  if (token.ACTIVE_ROBOTS == 0)
  {
    c_print("Impossibile allocare missioni anche con un solo robot", red, P);
    // mando il token al taskplanner segnalando la terminazione
    token.SHUTDOWN = true;
  }
  else if (ID_ROBOT < token.ACTIVE_ROBOTS)  // robot attivo
  {
    c_print("Allocazione task...", green, P);
    // prendo una missione tra quelle disponibili
    auto it = token.TAKEN_MISSION.begin();
    int i;
    for (; it != token.TAKEN_MISSION.end(); it++)
    {
      i = it - token.TAKEN_MISSION.begin();
      if (*it == -1)
      {
        break;
      }
    }

    // trovata una missione non assegnata
    if (it != token.TAKEN_MISSION.end())
    {
      *it = ID_ROBOT;
    }
    else if (ID_ROBOT == token.ACTIVE_ROBOTS - 1)  // tutte le missioni sono state assegnate
    {
      c_print("Missioni assegnate", green, P);
      token.ALLOCATE = false;
      token.GOOD_PATHS = true;

      token.CALCULATE_PATHS = true;
      for (int i = 0; i < token.ACTIVE_ROBOTS; i++)
      {
        token.NEW_TRAILS[i].PATH = token.TRAILS[i].PATH;
      }
    }
  }
}

void OnlineAgent::token_simple_planning(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  // caso speciale, un solo robot rimasto, tutti provano a pianfificare
  if (token.ACTIVE_ROBOTS == 1)
  {
    c_print("Pianificazione percorsi...", green, P);
    // creo il vettore di waypoints
    uint init_pos = initial_vertex;
    std::vector<uint> waypoints = { token.TRAILS[ID_ROBOT].PATH.back() };
    for (const logistic_sim::Mission &m : token.MISSION)
    {
      waypoints.push_back(src_vertex);
      for (uint dst : m.DSTS)
      {
        waypoints.push_back(dst);
      }
    }
    waypoints.push_back(init_pos);
    try
    {
      if (token.TRAILS[ID_ROBOT].PATH.empty())
      {
        c_print("[WARN]\tPercorso vuoto!", yellow, P);
      }

      std::vector<uint> path =
          spacetime_dijkstra(token.NEW_TRAILS, map_graph, waypoints, token.NEW_TRAILS[ID_ROBOT].PATH.size() - 1);
      token.NEW_TRAILS[ID_ROBOT].PATH.pop_back();
      token.NEW_TRAILS[ID_ROBOT].PATH.insert(token.NEW_TRAILS[ID_ROBOT].PATH.end(), path.begin(), path.end());

      // pusha i nuovi percorsi e aggiorna le statistiche
      c_print("Percorsi calcolati!", green, P);
      token.TRAILS[ID_ROBOT].PATH = token.NEW_TRAILS[ID_ROBOT].PATH;

      for (int i = 0; i < token.MISSION.size(); i++)
      {
        token.MISSIONS_COMPLETED[ID_ROBOT]++;
        token.TASKS_COMPLETED[ID_ROBOT] += token.MISSION[i].DEMANDS.size();
      }

      // segnalo al planner che sono stati distribuiti i task
      token.NEW_MISSIONS_AVAILABLE = false;
      token.CALCULATE_PATHS = false;
    }
    catch (const std::string &e)
    {
      c_print("Impossibile pianificare con ", token.ACTIVE_ROBOTS, red, P);
      token.GOOD_PATHS = false;
    }
  }
  // caso base calcolo percorsi
  else if (msg->GOOD_PATHS && ID_ROBOT < token.ACTIVE_ROBOTS)
  {
    c_print("Pianificazione percorsi...", green, P);

    // riempo vettore missions
    missions.clear();
    for (auto it = token.TAKEN_MISSION.begin(); it != token.TAKEN_MISSION.end(); it++)
    {
      if (*it == ID_ROBOT)
      {
        int index = it - token.TAKEN_MISSION.begin();
        missions.push_back(token.MISSION[index]);
      }
    }

    // creo il vettore di waypoints
    uint init_pos = initial_vertex;
    std::vector<uint> waypoints = { token.TRAILS[ID_ROBOT].PATH.back() };
    for (const logistic_sim::Mission &m : missions)
    {
      waypoints.push_back(src_vertex);
      for (uint dst : m.DSTS)
      {
        waypoints.push_back(dst);
      }
    }
    waypoints.push_back(init_pos);
    try
    {
      if (token.TRAILS[ID_ROBOT].PATH.empty())
      {
        c_print("[WARN]\tPercorso vuoto!", yellow, P);
      }

      if (!missions.empty())
      {
        std::vector<uint> path =
            spacetime_dijkstra(token.NEW_TRAILS, map_graph, waypoints, token.NEW_TRAILS[ID_ROBOT].PATH.size() - 1);
        token.NEW_TRAILS[ID_ROBOT].PATH.pop_back();
        token.NEW_TRAILS[ID_ROBOT].PATH.insert(token.NEW_TRAILS[ID_ROBOT].PATH.end(), path.begin(), path.end());
      }

      // l'ultimo robot attivo pusha i nuovi percorsi e aggiorna le statistiche
      if (ID_ROBOT == token.ACTIVE_ROBOTS - 1)
      {
        c_print("Percorsi calcolati!", green, P);
        for (int i = 0; i < token.ACTIVE_ROBOTS; i++)
        {
          token.TRAILS[i].PATH = token.NEW_TRAILS[i].PATH;
        }

        for (int i = 0; i < token.TAKEN_MISSION.size(); i++)
        {
          int id = token.TAKEN_MISSION[i];
          token.MISSIONS_COMPLETED[id]++;
          token.TASKS_COMPLETED[id] += token.MISSION[i].DEMANDS.size();
        }

        // segnalo al planner che sono stati distribuiti i task
        token.NEW_MISSIONS_AVAILABLE = false;
        token.CALCULATE_PATHS = false;
      }
    }
    catch (const std::string &e)
    {
      c_print("Impossibile pianificare con ", token.ACTIVE_ROBOTS, red, P);
      token.GOOD_PATHS = false;
    }
  }

  if (!token.GOOD_PATHS)
  {
    if (ID_ROBOT == TEAM_SIZE - 1)
    {
      token.ACTIVE_ROBOTS--;
      token.ALLOCATE = true;
      token.CALCULATE_PATHS = false;
      token.TAKEN_MISSION = std::vector<int>(token.MISSION.size(), -1);
    }
  }
}

void OnlineAgent::token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  // avanzamento dei robot

  // aggiorno posizioni dei robot nel token
  token.X_POS[ID_ROBOT] = xPos[ID_ROBOT];
  token.Y_POS[ID_ROBOT] = yPos[ID_ROBOT];
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (i != ID_ROBOT)
    {
      xPos[i] = token.X_POS[i];
      yPos[i] = token.Y_POS[i];
    }
  }

  // gestisco l'arrivo al goal
  if (goal_success)
  {
    // aggiorno distanza percorsa nel token
    uint edge_length = 0;

    if (current_vertex < dimension)
    {
      for (int i = 0; i < vertex_web[current_vertex].num_neigh; i++)
      {
        if (vertex_web[current_vertex].id_neigh[i] == next_vertex)
        {
          edge_length += vertex_web[current_vertex].cost[i];
          break;
        }
      }
    }
    token.TOTAL_DISTANCE[ID_ROBOT] += edge_length;

    if (token.TRAILS[ID_ROBOT].PATH.size() > 1)
    {
      token.TRAILS[ID_ROBOT].PATH.erase(token.TRAILS[ID_ROBOT].PATH.begin());
      if (token.TRAILS[ID_ROBOT].PATH.size() == 1 && !token.HOME_TRAILS[ID_ROBOT].PATH.empty())
      {
        token.TRAILS[ID_ROBOT].PATH.push_back(token.HOME_TRAILS[ID_ROBOT].PATH.front());
        token.HOME_TRAILS[ID_ROBOT].PATH.erase(token.HOME_TRAILS[ID_ROBOT].PATH.begin());
      }
    }
    token.GOAL_STATUS[ID_ROBOT]++;

    // exit check routine
    if (!token.NEW_MISSIONS_AVAILABLE && token.ALL_MISSIONS_INSERTED && token.TRAILS[ID_ROBOT].PATH.size() == 1)
    {
      token.REACHED_HOME[ID_ROBOT] = true;
      bool can_exit = true;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        if (!token.REACHED_HOME[i])
        {
          can_exit = false;
          break;
        }
      }

      if (can_exit)
      {
        c_print("Finiti tutti i task, esco!", magenta, P);
        token.SHUTDOWN = true;
      }
    }

    current_vertex = next_vertex;
    goal_success = false;
    still = true;
  }

  bool equal_status = true;
  int status = token.GOAL_STATUS[0];
  for (int i = 1; i < TEAM_SIZE; i++)
  {
    if (token.GOAL_STATUS[i] != status)
      equal_status = false;
  }

  if (equal_status && still)
  {
    // if new missions are avaiable they must be allocated
    // it's essential that robots are synchronized in terms of goal
    // otherwise paths would not be synched and the planning would not be valid
    bool first_to_see_equal = false;
    status = msg->GOAL_STATUS[0];
    for (int i = 1; i < TEAM_SIZE; i++)
    {
      if (msg->GOAL_STATUS[i] != status)
        first_to_see_equal = true;
    }

    if (msg->NEW_MISSIONS_AVAILABLE && first_to_see_equal)
    {
      token.ALLOCATE = true;

      // send the token to the robot with the shortest path
      int id_next_robot = 0;
      int min_path_len = token.TRAILS[0].PATH.size();
      for (int i = 1; i < TEAM_SIZE; i++)
      {
        int len = token.TRAILS[i].PATH.size();
        if (len < min_path_len)
        {
          min_path_len = len;
          id_next_robot = i;
        }
      }

      token.ID_RECEIVER = id_next_robot;
    }
    else
    {
      if (token.TRAILS[ID_ROBOT].PATH.size() > 1)
      {
        next_vertex = token.TRAILS[ID_ROBOT].PATH[1];
      }
      else
      {
        c_print("[ WARN]Don't know what to do!!!", yellow, P);
        next_vertex = current_vertex;
      }

      still = false;
      c_print("before OnGoal()", magenta);
      c_print("[DEBUG]\tGoing to ", next_vertex, green, P);
      sendGoal(next_vertex);
    }
  }

  // put in the token the current edge
  token.CURR_VERTEX[ID_ROBOT] = current_vertex;
  token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
}

void OnlineAgent::token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
{
  /* uso solo fase ALLOCATE
   * the multi-item tasks in the window are allocated one by one
   * the token is sent to the robot with the shortest path, if such
   * robot can't find a valid planning the token is sent to the robot with
   * the second shortest path and so on until the task is not allocated
   * (the last robot will always be able to allocate such task)
   */

  // planner has sent the token to the robot with the shortest path
  logistic_sim::Mission m = token.MISSION.front();

  // merge task path with return to home path
  std::vector<logistic_sim::Path> robot_paths = token.TRAILS;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    robot_paths[i].PATH.insert(robot_paths[i].PATH.end(), token.HOME_TRAILS[i].PATH.begin(),
                               token.HOME_TRAILS[i].PATH.end());
  }

  int id_next_robot;
  int min_length;

  try
  {
    std::vector<unsigned int> last_leg, first_leg;
    std::vector<unsigned int> waypoints;

    waypoints.push_back(token.TRAILS[ID_ROBOT].PATH.back());
    waypoints.push_back(src_vertex);
    for (auto dst : m.DSTS)
    {
      waypoints.push_back(dst);
    }
    waypoints.push_back(initial_vertex);
    auto f = boost::bind(astar_cmp_function, min_hops_matrix, waypoints, _1, _2);
    std::vector<unsigned int> astar_result = spacetime_dijkstra(
        robot_paths, map_graph, waypoints, token.TRAILS[ID_ROBOT].PATH.size() - 1, &last_leg, &first_leg, &f);
    std::cout << "first_leg: ";
    for (unsigned int v : first_leg)
    {
      std::cout << v << " ";
    }
    std::cout << "\nlast_leg: ";
    for (unsigned int v : last_leg)
    {
      std::cout << v << " ";
    }
    std::cout << std::endl;
    token.TRAILS[ID_ROBOT].PATH.pop_back();
    token.TRAILS[ID_ROBOT].PATH.insert(token.TRAILS[ID_ROBOT].PATH.end(), first_leg.begin(), first_leg.end());
    token.HOME_TRAILS[ID_ROBOT].PATH = last_leg;

    // update task + home path (to find who has the shortest one between all robots)
    robot_paths[ID_ROBOT].PATH = token.TRAILS[ID_ROBOT].PATH;
    robot_paths[ID_ROBOT].PATH.insert(robot_paths[ID_ROBOT].PATH.end(), token.HOME_TRAILS[ID_ROBOT].PATH.begin(),
                                      token.HOME_TRAILS[ID_ROBOT].PATH.end());

    // update token statistics
    token.MISSIONS_COMPLETED[ID_ROBOT]++;
    token.TASKS_COMPLETED[ID_ROBOT] += m.DEMANDS.size();

    // remove mission from token
    token.MISSION.erase(token.MISSION.begin());

    // booleans in ros msgs becomes uint8_t
    token.FAILED_ALLOCATION = std::vector<uint8_t>(TEAM_SIZE, false);

    // if the robot has received this task after it has reached home
    // this prevents premature shutdown
    token.REACHED_HOME[ID_ROBOT] = false;

    // if there are still missions to assign the token is sent
    // to the robot with the shortest path
    if (!token.MISSION.empty())
    {
      id_next_robot = 0;
      min_length = token.TRAILS[0].PATH.size();
      for (int i = 1; i < TEAM_SIZE; i++)
      {
        int len = token.TRAILS[i].PATH.size();
        if (len < min_length)
        {
          min_length = len;
          id_next_robot = i;
        }
      }

      token.ID_RECEIVER = id_next_robot;
    }
    else
    {
      // if all tasks have been assigned the token
      // is sent to the planner in case new tasks
      // have been generated
      token.ALLOCATE = false;
      token.NEW_MISSIONS_AVAILABLE = false;
      token.ID_RECEIVER = TASK_PLANNER_ID;
    }
  }
  catch (std::string &e)
  {
    c_print("Can't find a valid path", yellow, P);

    token.FAILED_ALLOCATION[ID_ROBOT] = true;

    // search the next robot
    min_length = std::numeric_limits<int>::max();
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      if (!token.FAILED_ALLOCATION[i] && robot_paths[i].PATH.size() < min_length)
      {
        min_length = robot_paths[i].PATH.size();
        id_next_robot = i;
      }
    }

    token.ID_RECEIVER = id_next_robot;
  }
}


void OnlineAgent::update_graph()
{
  // for now just rebuild the graph from scratch
  map_graph = build_graph();
  min_hops_matrix = calculate_min_hops_matrix();
}


std::vector<std::vector<unsigned int> > OnlineAgent::build_graph()
{
  std::vector<std::vector<unsigned int> > result = std::vector<std::vector<unsigned int>>(dimension);
  for (int i = 0; i < dimension; i++)
  {
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      for (int k = 0; k < dimension; k++)
      {
        if (vertex_web[k].id == vertex_web[i].id_neigh[j]) {
          result[i].push_back(k);
          break;
        }
      }
    }
  }

  return result;
}


/*
 * Floyd-Warshall to find the least number of hops between each pair of vertices
 */
std::vector<std::vector<unsigned int>> OnlineAgent::calculate_min_hops_matrix()
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

}  // namespace onlineagent