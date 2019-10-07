#include "CFreeAgent.hpp"

using namespace cfreeagent;

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
    p.PATH = std::vector<uint>(10, initial_vertex);
    token.TRAILS.push_back(p);
    token.REACHED_HOME.push_back(false);

    initialize = false;
    if (ID_ROBOT == TEAM_SIZE - 1)
    {
      token.STEP = true;
    }
  }
  else if (msg->STEP)
  {
    std::cout << "Inizio fase step" << std::endl;
    // static std::vector<logistic_sim::Mission> missions;
    if (!token.MISSION.empty())
    {
      if (missions.empty())
      {
        logistic_sim::Mission m = token.MISSION.back();
        token.MISSION.pop_back();
        missions.emplace(missions.begin(), m);
      }
      else
      {
        bool taken = false;
        auto last_mission = missions.back();
        auto min_dst = *std::min_element(last_mission.DSTS.begin(), last_mission.DSTS.end());
        for (int i = 0; i < token.MISSION.size(); i++)
        {
          auto tmp_min_dst = *std::min_element(token.MISSION[i].DSTS.begin(), token.MISSION[i].DSTS.end());
          if (min_dst <= tmp_min_dst)
          {
            logistic_sim::Mission m1 = token.MISSION[i];
            missions.emplace(missions.begin(), m1);
            token.MISSION.erase(token.MISSION.begin() + i);
            taken = true;
            break;
          }
        }

        // se l'euristica non va bene prendo un'altra
        if (!taken)
        {
          logistic_sim::Mission m1 = token.MISSION.back();
          token.MISSION.pop_back();
          missions.emplace(missions.begin(), m1);
          taken = true;
        }
      }
    }

    // PRINT TOKEN MISSIONS
    // std::cout << "=======================" << std::endl;
    // for(auto it = token.MISSION.begin(); it != token.MISSION.end(); it++)
    // {
    //     const logistic_sim::Mission &m = *it;
    //     std::cout << "Mission id: " << m.ID << std::endl;
    //     for(auto jt = m.DSTS.begin(); jt != m.DSTS.end(); jt++)
    //     {
    //         std::cout << *jt << " ";
    //     }
    //     std::cout << std::endl << std::endl;
    // }
    if (token.MISSION.empty() && ID_ROBOT == TEAM_SIZE - 1)
    {
      std::cout << "Tutti hanno preso le missioni" << std::endl;
      token.STEP = false;
    }
  }
  else
  {
    if (!path_calculated)
    {
      std::cout << "Calcolo percorso..." << std::endl;
      uint init_pos = 9;
      if (mapname == "grid")
      {
        init_pos = initial_vertex;
      }
      std::vector<uint> waypoints = { current_vertex };
      for (logistic_sim::Mission m : missions)
      {
        token.MISSIONS_COMPLETED[ID_ROBOT]++;
        waypoints.push_back(src_vertex);
        for(uint demands : m.DEMANDS)
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
        token.TRAILS[ID_ROBOT].PATH = token_dijkstra(waypoints, token.TRAILS);
      }
      catch (std::string& e)
      {
        std::cout << e << std::endl;
        token.END_SIMULATION = true;
        token.MISSIONS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
        token.TASKS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      }

      std::cout << "Percorso robot " << ID_ROBOT << std::endl;
      for (uint v : token.TRAILS[ID_ROBOT].PATH)
      {
        std::cout << setw(2) << v << " ";
      }
      std::cout << std::endl;

      // entra solo l'ultimo robot a calcolare i percorsi per model6
      if (ID_ROBOT == TEAM_SIZE - 1 && mapname == "model6")
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
          token.TRAILS[indices[j]].PATH.insert(token.TRAILS[indices[j]].PATH.end(), 50, home_vertex);
          std::cout << std::endl;
        }
      }
      else if(mapname != "model6")
      {
        // se non siamo in model6 ogni robot si aggiorna il proprio percorso da solo
        token.TRAILS[ID_ROBOT].PATH.insert(token.TRAILS[ID_ROBOT].PATH.end(), 50, token.TRAILS[ID_ROBOT].PATH.back());
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
    }
    else
    {
      static bool first_round = true;
      if (first_round)
      {
        home_steps = token.TRAILS[ID_ROBOT].PATH.size() - 1 - 49;
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
