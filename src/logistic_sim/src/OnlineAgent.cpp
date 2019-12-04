#include "OnlineAgent.hpp"

namespace onlineagent
{
void OnlineAgent::init(int argc, char **argv)
{
  CFreeAgent::init(argc, argv);
  allocate_memory();
  map_graph = std::vector<std::vector<unsigned int>>(dimension);
  for (int i = 0; i < dimension; i++)
  {
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      map_graph[i].push_back(vertex_web[i].id_neigh[j]);
    }
  }
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
    c_print("Inizializzazione...", green, P);
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
    token.REACHED_HOME.push_back(false);
    token.ACTIVE_ROBOTS = TEAM_SIZE;

    // test
    // if (ID_ROBOT == 3)
    // {
    // token.TRAILS[0].PATH = { 1, 6, 11, 10 };
    // token.TRAILS[1].PATH = { 3, 8, 13, 14 };
    // token.TRAILS[2].PATH = { 21, 16, 16, 11};
    // token.TRAILS[3].PATH = { 23, 18, 18, 13};
    // }

    initialize = false;
    next_vertex = current_vertex = initial_vertex;
    goal_complete = true;
  }

  if (msg->CALCULATE_PATHS)
  {
    if (msg->GOOD_PATHS && ID_ROBOT < token.ACTIVE_ROBOTS)
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
      for (logistic_sim::Mission &m : missions)
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

        // l'ultimo robot attivo pusha i nuovi percorsi
        if (ID_ROBOT == token.ACTIVE_ROBOTS - 1)
        {
          c_print("Percorsi calcolati!", green, P);
          for (int i = 0; i < token.ACTIVE_ROBOTS; i++)
          {
            token.TRAILS[i].PATH = token.NEW_TRAILS[i].PATH;
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
  else if (msg->ALLOCATE)
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
  else
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
        if (token.ALL_MISSIONS_INSERTED && token.TRAILS[ID_ROBOT].PATH.size() == 1)
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
          if (false)
          // if (can_exit)
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

  ros::Duration(0.03).sleep();
  token_pub.publish(token);
  ros::spinOnce();

  if (token.SHUTDOWN)
  {
    c_print("Segnale di shutdown, termino...", magenta, P);
    ros::shutdown();
  }
}

// genera un percorso fino all'ultimo waypoint
std::vector<unsigned int> OnlineAgent::spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                                          const std::vector<std::vector<unsigned int>> &graph,
                                                          const std::vector<unsigned int> &waypoints, int start_time)
{
  std::cout << "SPACETIME DIJKSTRA --- WAYPOINTS:";
  for (int i = 0; i < waypoints.size(); i++)
  {
    std::cout << " " << waypoints[i];
  }
  std::cout << std::endl;

  unsigned int source = waypoints.front();
  auto it_waypoints = waypoints.begin() + 1;

  // inizializzazione strutture
  for (unsigned int i = 0; i < graph.size(); i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        path_sizes[i][j][k] = 0;
      }
    }
  }
  prev_paths[source][0][1][0] = source;
  path_sizes[source][0][1] = 1;

  for (unsigned int i = 0; i < graph.size(); i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        visited[i][j][k] = WHITE;
      }
    }
  }

  // inizializzazione coda
  for (unsigned int i = 0; i < MAX_TIME; i++)
  {
    queue[i] = st_location();
  }
  st_location st(source, start_time, 1);
  queue[0] = st;
  int queue_size = 1;

  // individuo la lunghezza del percorso maggiore
  uint max_path_size = 0;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (i != ID_ROBOT && other_paths[i].PATH.size() > max_path_size)
    {
      max_path_size = other_paths[i].PATH.size();
    }
  }

  while (queue_size > 0)
  {
    st_location current_st = queue[--queue_size];
    unsigned int u = current_st.vertex;
    unsigned int time = current_st.time - start_time;
    unsigned int current_waypoint = current_st.waypoint;
    unsigned int next_next_waypoint;

    visited[u][time][current_waypoint] = BLACK;

    if (u == waypoints[current_waypoint])
    {
      if (current_waypoint + 1 == waypoints.size())
      {
        if (time + 1 + start_time >= max_path_size)
        {
          std::vector<unsigned int> result =
              std::vector<unsigned int>(prev_paths[u][time][current_waypoint],
                                        prev_paths[u][time][current_waypoint] + path_sizes[u][time][current_waypoint]);

          return result;
        }
        else
        {
          next_next_waypoint = current_waypoint;
        }
      }
      else
      {
        next_next_waypoint = current_waypoint + 1;
      }
    }
    else  // u non è waypoint
    {
      next_next_waypoint = current_waypoint;
    }

    // considero l'attesa sul posto
    unsigned int next_time = time + 1;
    if (next_time < MAX_TIME && visited[u][next_time][next_next_waypoint] == WHITE)
    {
      bool good = true;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        if (i != ID_ROBOT)
        {
          if (next_time + start_time < other_paths[i].PATH.size())
          {
            if (other_paths[i].PATH[next_time + start_time] == u)
            {
              // std::cout << "can't stand in " << u << " at time " << time << std::endl;
              good = false;
              break;
            }
          }
          else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
          {
            if (other_paths[i].PATH.back() == u)
            {
              // std::cout << "can't stand in " << u << " at time " << time << std::endl;
              good = false;
              break;
            }
          }
        }
      }

      if (good)
      {
        st_location next_st(u, next_time + start_time, next_next_waypoint);
        queue_size = insertion_sort(queue, queue_size, next_st);
        visited[u][next_time][next_next_waypoint] = GRAY;

        unsigned int psize = path_sizes[u][time][current_waypoint];
        for (unsigned int i = 0; i < psize; i++)
        {
          prev_paths[u][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
        }
        prev_paths[u][next_time][next_next_waypoint][psize] = u;
        path_sizes[u][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
      }
    }

    // considero i vicini
    for (auto it = graph[u].begin(); it != graph[u].end(); it++)
    {
      const unsigned int v = *it;
      const unsigned int next_time = time + 1;

      if (next_time < MAX_TIME && visited[v][next_time][next_next_waypoint] == WHITE)
      {
        bool good = true;
        for (int i = 0; i < TEAM_SIZE; i++)
        {
          if (i != ID_ROBOT)
          {
            if (next_time + start_time < other_paths[i].PATH.size())
            {
              if (other_paths[i].PATH[next_time + start_time] == v)
              {
                // std::cout << "can't go in " << v << " at time " << time << std::endl;
                good = false;
                break;
              }
              if (other_paths[i].PATH[next_time + start_time] == u && other_paths[i].PATH[time + start_time] == v)
              {
                // std::cout << "can't go in " << v << " at time " << time << std::endl;
                good = false;
                break;
              }
            }
            else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
            {
              if (other_paths[i].PATH.back() == v)
              {
                // std::cout << "can't go in " << v << " at time " << time << ", there is a still robot" << std::endl;
                good = false;
                break;
              }
            }
          }
        }

        if (good)
        {
          st_location next_st(v, next_time + start_time, next_next_waypoint);
          queue_size = insertion_sort(queue, queue_size, next_st);
          visited[v][next_time][next_next_waypoint] = GRAY;

          unsigned int psize = path_sizes[u][time][current_waypoint];
          for (unsigned int i = 0; i < psize; i++)
          {
            prev_paths[v][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
          }
          prev_paths[v][next_time][next_next_waypoint][psize] = v;
          path_sizes[v][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
        }
      }
    }
  }

  throw std::string("Can't find path!!!");
}

}  // namespace onlineagent

int main(int argc, char *argv[])
{
  onlineagent::OnlineAgent OA;
  OA.init(argc, argv);
  c_print("@ ONLINE", green);
  sleep(3);
  OA.run();
  return 0;
}