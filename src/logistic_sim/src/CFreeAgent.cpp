#include "CFreeAgent.hpp"

using namespace cfreeagent;

int taken_missions_count(const logistic_sim::Token& token)
{
  int count = 0;
  for (int i = 0; i < token.TAKEN_MISSION.size(); i++)
  {
    if (token.TAKEN_MISSION[i] != -1)
      count++;
  }
  return count;
}

void CFreeAgent::token_callback(const logistic_sim::TokenConstPtr& msg)
{
  // ricevo il token ricevo il task set computo la CF migliore la assegno e toglo i task che la compongono.

  // se non è per me termino
  if (msg->ID_RECEIVER != ID_ROBOT)
    return;

  logistic_sim::Token token;
  token = *msg;

  // logistic_sim::Mission m = coalition_formation(token);

  // cout << "missione finale dopo coalizione: "<< m.ID << m.TOT_DEMAND <<"\n";

  token.ID_SENDER = ID_ROBOT;
  // c_print("TEAMSIZE: ",TEAM_SIZE, yellow);
  if (msg->ID_RECEIVER == TEAM_SIZE - 1)
  {
    token.ID_RECEIVER = TASK_PLANNER_ID;
  }
  else
  {
    token.ID_RECEIVER = (ID_ROBOT + 1) % TEAM_SIZE;
  }

  if (msg->CHECK)
  {
    int id_vertex_stuck;
    // controlliamo che non ci siano situazioni ti stuck lungo tutti i percorsi calcolati
    for (int i = 0; i < TEAM_SIZE; i++)
    {
      if (!token_check_pt(token.TRAILS[i].PATH, token.TRAILS, i, &id_vertex_stuck))
      {
        c_print("Non sono CFree i percorsi calcolati per ID: ", i, " vertex: ", id_vertex_stuck, red, P);
        token.END_SIMULATION = true;
      }
    }
    token.CHECK = false;
  }

  if (msg->INIT)
  {
    allocate_memory();
    token.CAPACITY.push_back(CAPACITY);
    // all avvio indico quale arco i robot vorrebbero attraversare
    // serve a forzare la partenza nella stessa direzione
    if (ID_ROBOT > 0)
    {
      init_next_vertex = token.CURR_VERTEX[ID_ROBOT - 1];
    }
    else
    {
      init_next_vertex = current_vertex;
    }
    token.CURR_VERTEX.push_back(current_vertex);
    token.NEXT_VERTEX.push_back(init_next_vertex);
    // solo INIT
    token.CURR_DST.push_back(dimension + 1);
    token.INIT_POS.insert(token.INIT_POS.begin(), initial_vertex);
    token.INIT_POS_INDEX = 0;
    token.MISSION_START_TIME.push_back(ros::Time::now());
    token.MISSION_CURRENT_DISTANCE.push_back(0.0f);
    token.INTERFERENCE_COUNTER.push_back(0);
    token.MISSIONS_COMPLETED.push_back(0);
    token.TASKS_COMPLETED.push_back(0);
    token.TOTAL_DISTANCE.push_back(0.0f);
    token.INTERFERENCE_STATUS.push_back(0);
    token.X_POS.push_back(0.0);
    token.Y_POS.push_back(0.0);
    token.GOAL_STATUS.push_back(0);
    logistic_sim::Path p;
    // p.PATH = std::vector<uint>(10, initial_vertex);
    token.TRAILS.push_back(p);
    if (ID_ROBOT == TEAM_SIZE - 1)
    {
      token.STEP = true;
    }
    token.REACHED_HOME.push_back(false);
    token.ACTIVE_ROBOTS = TEAM_SIZE;

    initialize = false;
  }
  else if (msg->STEP)
  {
    std::cout << "Inizio fase step" << std::endl;
    // static std::vector<logistic_sim::Mission> missions;
    // if (!token.MISSION.empty())

    std::cout << "robot attivi: " << token.ACTIVE_ROBOTS << std::endl;
    if (token.ACTIVE_ROBOTS == 0)
    {
      std::cout << "Impossibile trovare percorso anche con un solo robot!!!" << std::endl;
      token.END_SIMULATION = true;
      token.MISSIONS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      token.TASKS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      token.INIT_POS.clear();
      ros::shutdown();
      system("./stop_experiment.sh");
    }
    else if (ID_ROBOT < token.ACTIVE_ROBOTS) // guardo se sono tra i robot attivi
    {
      
      int count = 0;
      for (int i = 0; i < token.MISSION.size(); i++)
      {
        if (token.TAKEN_MISSION[i] == ID_ROBOT)
          count++;
      }
      // se non ho preso missioni pulisco il mio vettore di missioni
      if (count == 0)
      {
        missions.clear();
      }

      // guardo se ci sono ancora missioni da prendere
      if (taken_missions_count(token) < token.MISSION.size())
      {
        logistic_sim::Mission m;
        for (int i = 0; i < token.MISSION.size(); i++)
        {
          if (token.TAKEN_MISSION[i] == -1)
          {
            m = token.MISSION[i];
            token.TAKEN_MISSION[i] = ID_ROBOT;
            break;
          }
        }
        missions.emplace(missions.begin(), m);
        // logistic_sim::Mission m = token.MISSION.back();
        // token.MISSION.pop_back();
        // if (missions.empty())
        // {
        // }
        // else
        // {
        //   bool taken = false;
        //   auto last_mission = missions.back();
        //   auto min_dst = *std::min_element(last_mission.DSTS.begin(), last_mission.DSTS.end());
        //   for (int i = 0; i < token.MISSION.size(); i++)
        //   {
        //     auto tmp_min_dst = *std::min_element(token.MISSION[i].DSTS.begin(), token.MISSION[i].DSTS.end());
        //     if (min_dst <= tmp_min_dst)
        //     {
        //       logistic_sim::Mission m1 = token.MISSION[i];
        //       missions.emplace(missions.begin(), m1);
        //       token.MISSION.erase(token.MISSION.begin() + i);
        //       taken = true;
        //       break;
        //     }
        //   }

        //   // se l'euristica non va bene prendo un'altra
        //   if (!taken)
        //   {
        //     logistic_sim::Mission m1 = token.MISSION.back();
        //     token.MISSION.pop_back();
        //     missions.emplace(missions.begin(), m1);
        //     taken = true;
        //   }
        // }
      }
    }
    else
    {
      token.TRAILS[ID_ROBOT].PATH = std::vector<uint>(1, current_vertex);
    }

    if (taken_missions_count(token) == token.MISSION.size() && ID_ROBOT == TEAM_SIZE - 1)
    {
      std::cout << "Tutti hanno preso le missioni" << std::endl;
      token.STEP = false;
      token.CALCULATE_PATHS = true;
      token.GOOD_PATHS = true;
    }
  }
  else if (token.CALCULATE_PATHS)
  {
    if (token.GOOD_PATHS)
    {
      // se sono tra i robot attivi
      if (ID_ROBOT < token.ACTIVE_ROBOTS)
      {
        // preparo i waypoints
        std::cout << "Calcolo percorso..." << std::endl;
        token.TRAILS[ID_ROBOT].PATH.clear();
        uint init_pos = 9;
        if (mapname != "model6")
        {
          init_pos = initial_vertex;
        }
        std::vector<uint> waypoints = { current_vertex };
        for (logistic_sim::Mission m : missions)
        {
          token.MISSIONS_COMPLETED[ID_ROBOT]++;
          waypoints.push_back(src_vertex);
          for (uint demands : m.DEMANDS)
          {
            token.TASKS_COMPLETED[ID_ROBOT]++;
          }
          for (uint dst : m.DSTS)
          {
            waypoints.push_back(dst);
          }
        }
        waypoints.push_back(init_pos);
        try
        {
          // preparo vettore robot inattivi per dijkstra
          std::vector<bool> still_robots(TEAM_SIZE, true);
          for (int i = 0; i < token.ACTIVE_ROBOTS; i++)
          {
            still_robots[i] = false;
          }
          token.TRAILS[ID_ROBOT].PATH = token_dijkstra(waypoints, token.TRAILS, still_robots);
          std::cout << "Percorso robot " << ID_ROBOT << std::endl;
          for (uint v : token.TRAILS[ID_ROBOT].PATH)
          {
            std::cout << setw(2) << v << " ";
          }
          std::cout << std::endl;

          // entra solo l'ultimo robot a calcolare i pezzi finali per model6
          if (ID_ROBOT == token.ACTIVE_ROBOTS - 1 && mapname == "model6")
          {
            std::vector<uint> indices(TEAM_SIZE);
            std::cout << "Dimensione percorsi\n";
            for (int i = 0; i < TEAM_SIZE; i++)
            {
              std::cout << "robot " << i << ": " << token.TRAILS[i].PATH.size() << std::endl;
              indices[i] = i;
            }
            auto cmp_function = [&](uint lhs, uint rhs) {
              return token.TRAILS[lhs].PATH.size() < token.TRAILS[rhs].PATH.size();
            };
            std::sort(indices.begin(), indices.end(), cmp_function);
            for (int j = 0; j < indices.size(); j++)
            {
              // j indica il nodo home
              // indices[j] indica il robot assegnato a quel nodo
              std::cout << "Casa robot " << indices[j] << "\n";
              int home_vertex = j;
              for (int i = 5; i >= home_vertex; i--)
              {
                std::cout << i << " ";
                token.TRAILS[indices[j]].PATH.push_back(i);
              }
              // token.TRAILS[indices[j]].PATH.insert(token.TRAILS[indices[j]].PATH.end(), 50, home_vertex);
              std::cout << std::endl;
            }
          }
          else if (mapname != "model6")
          {
            // se non siamo in model6 ogni robot si aggiorna il proprio percorso da solo
            // token.TRAILS[ID_ROBOT].PATH.insert(token.TRAILS[ID_ROBOT].PATH.end(), 50,
            //                                    token.TRAILS[ID_ROBOT].PATH.back());
          }

          for (int j = 0; j < TEAM_SIZE; j++)
          {
            std::cout << "Percorso robot " << j << std::endl;
            for (uint v : token.TRAILS[j].PATH)
            {
              std::cout << setw(2) << v << " ";
            }
            std::cout << std::endl;
          }

          token.CHECK = true;

          // sendGoal(token.TRAILS[ID_ROBOT].PATH[0]);
          goal_complete = true;
          path_calculated = true;
          if (ID_ROBOT == TEAM_SIZE - 1)
          {
            token.STEP = false;
            token.CALCULATE_PATHS = false;
          }
        }
        catch (std::string& e) // se il dijkstra è fallito devo togliere un robot
        {
          token.GOOD_PATHS = false;
          for (int i = 0; i < token.MISSION.size(); i++)
          {
            c_print("Can't plan with ", token.ACTIVE_ROBOTS, " trying with one less", yellow, P);
            // resetto le missioni prese
            token.TAKEN_MISSION[i] = -1;
            token.ACTIVE_ROBOTS--;
          }

          // se sono l'ultimo robot devo attivare la fase STEP
          if (ID_ROBOT == TEAM_SIZE - 1)
          {
            token.STEP = true;
            token.CALCULATE_PATHS = false;
          }
          // std::cout << e << std::endl;
          // token.END_SIMULATION = true;
          // token.MISSIONS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
          // token.TASKS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
        }
      }
      else  // sono uno dei robot inattivi
      {
        // estendo il vettore di nodi con il vertice iniziale
        // posso farlo qui perchè i robot inattivi vengono
        // sempre dopo quelli attivi (rispetto all'ordine del token)
        int max_path=0;
        for(int i=0; i<token.ACTIVE_ROBOTS; i++)
        {
          if (token.TRAILS[i].PATH.size() > max_path)
          {
            max_path = token.TRAILS[i].PATH.size();
          }
        }
        token.TRAILS[ID_ROBOT].PATH = std::vector<uint>(max_path, current_vertex);
      }
    }
    else // un robot precedente non ha trovato un percorso valido
    {
      // se sono l'ultimo attivo la fase STEP
      if (ID_ROBOT == TEAM_SIZE - 1)
      {
        token.STEP = true;
        token.CALCULATE_PATHS = false;
      }
    }
  }
  else // fase di esecuzione percorsi
  {
    // if (!path_calculated)
    // {
    // }
    // else
    {
      static bool first_round = true;
      if (first_round)
      {
        home_steps = token.TRAILS[ID_ROBOT].PATH.size(); /* - 1 - 49; */
        first_round = false;
      }
      // aggiorno posizione
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

      // metto nel token quale arco sto occupando
      token.CURR_VERTEX[ID_ROBOT] = current_vertex;
      token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
      if (!current_mission.DSTS.empty())
      {
        token.CURR_DST[ID_ROBOT] = current_mission.DSTS[0];
      }
      else
      {
        token.CURR_DST[ID_ROBOT] = initial_vertex;
      }

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

          // controllo se sto andando a casa
          home_count++;
          // sono arrivato
          if (home_count == home_steps)
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
              token.INIT_POS.clear();
            }
          }

          first_time = false;
        }
        current_vertex = next_vertex;

        bool equal_status = true;
        int status = token.GOAL_STATUS[0];
        for (int i = 1; i < TEAM_SIZE; i++)
        {
          if (token.GOAL_STATUS[i] != status)
            equal_status = false;
        }

        if (equal_status)
        {
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
    }
  }

  usleep(30000);
  token_pub.publish(token);
  ros::spinOnce();

  if (token.END_SIMULATION)
  {
    c_print("Ho finito, bye bye", green, P);
    end_simulation = true;
  }
}  // token_callback()

int main(int argc, char* argv[])
{
  cfreeagent::CFreeAgent CFA;
  CFA.init(argc, argv);
  c_print("@ SPART", green);
  sleep(3);
  CFA.run();
  return 0;
}
