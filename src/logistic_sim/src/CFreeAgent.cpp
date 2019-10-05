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
    token.TRAILS.push_back(logistic_sim::Path());

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
        std::vector<uint> waypoints = { initial_vertex, src_vertex };
        for (auto dst : m.DSTS)
        {
          waypoints.push_back(dst);
        }
        std::vector<uint> path = token_dijkstra(waypoints, token.TRAILS);
        token.TRAILS[ID_ROBOT].PATH = path;
        current_mission = m;
        for (uint v : token.TRAILS[ID_ROBOT].PATH)
        {
          std::cout << setw(2) << v << " ";
        }
        std::cout << std::endl;
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
            try
            {
              logistic_sim::Mission m1 = token.MISSION[i];
              std::vector<uint> waypoints = { initial_vertex, src_vertex };
              for (auto dst : m1.DSTS)
              {
                waypoints.push_back(dst);
              }
              std::vector<uint> path = token_dijkstra(waypoints, token.TRAILS);
              missions.emplace(missions.begin(), m1);
              token.MISSION.erase(token.MISSION.begin() + i);
              current_mission = m1;
              taken = true;
              token.TRAILS[ID_ROBOT].PATH = path;
              for (uint v : token.TRAILS[ID_ROBOT].PATH)
              {
                std::cout << setw(2) << v << " ";
              }
              std::cout << std::endl;
              break;
            }
            catch (const std::string& e)
            {
              std::cerr << e << '\n';
            }
          }
        }

        if (!taken)
        {
          for (int i = 0; i < token.MISSION.size(); i++)
          {
            try
            {
              logistic_sim::Mission m1 = token.MISSION[i];
              std::vector<uint> waypoints = { initial_vertex, src_vertex };
              for (auto dst : m1.DSTS)
              {
                waypoints.push_back(dst);
              }
              std::vector<uint> path = token_dijkstra(waypoints, token.TRAILS);
              missions.emplace(missions.begin(), m1);
              token.MISSION.erase(token.MISSION.begin() + i);
              current_mission = m1;
              taken = true;
              token.TRAILS[ID_ROBOT].PATH = path;
              for (uint v : token.TRAILS[ID_ROBOT].PATH)
              {
                std::cout << setw(2) << v << " ";
              }
              std::cout << std::endl;
              break;
            }
            catch (const std::string& e)
            {
              std::cerr << e << '\n';
            }
          }
        }

        if (!taken)
        {
          token.END_SIMULATION = true;
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
      if (ID_ROBOT == TEAM_SIZE - 1)
      {
        std::cout << "Tutti hanno preso le missioni" << std::endl;
        token.STEP = false;
      }
    }
  }
  else
  {
    if (!path_calculated)
    {
      // std::cout << "Calcolo percorso..." << std::endl;
      // uint init_pos = 9;
      // if (mapname == "grid")
      // {
      //   init_pos = initial_vertex;
      // }
      // std::vector<uint> waypoints = { current_vertex };
      // for (logistic_sim::Mission m : missions)
      // {
      //   waypoints.push_back(src_vertex);
      //   for (uint dst : m.DSTS)
      //   {
      //     waypoints.push_back(dst);
      //   }
      // }
      // waypoints.push_back(init_pos);
      // token_dijkstra(waypoints, token.TRAILS);
      // std::cout << "Percorso robot " << ID_ROBOT << std::endl;
      // for (uint v : token.TRAILS[ID_ROBOT].PATH)
      // {
      //   std::cout << setw(2) << v << " ";
      // }
      // std::cout << std::endl;

      // controllo se percorsi sono definiti per ritorno a casa
      // bool defined = true;
      // for (int i = 0; i < TEAM_SIZE; i++)
      // {
      //   if (token.TRAILS[i].PATH.empty())
      //     defined = false;
      // }
      // // entra solo l'ultimo robot a calcolare i percorsi
      // if (defined)
      // {
      //   // fisso la parte finale per tutti i robot
      //   if (mapname == "model6")
      //   {
      //     std::vector<uint> indices(TEAM_SIZE);
      //     std::cout << "Dimensione percorsi\n";
      //     for (int i = 0; i < TEAM_SIZE; i++)
      //     {
      //       std::cout << "robot " << i << ": " << token.TRAILS[i].PATH.size() << std::endl;
      //       indices[i] = i;
      //     }
      //     auto cmp_function = [&](uint lhs, uint rhs) {
      //       return token.TRAILS[lhs].PATH.size() < token.TRAILS[rhs].PATH.size();
      //     };
      //     std::sort(indices.begin(), indices.end(), cmp_function);
      //     for (int j = 0; j < indices.size(); j++)
      //     {
      //       // j indica il nodo home
      //       // indices[j] indica il robot assegnato a quel nodo
      //       std::cout << "Casa robot " << indices[j] << "\n";
      //       int home_vertex = j;
      //       for (int i = 5; i >= home_vertex; i--)
      //       {
      //         std::cout << i << " ";
      //         token.TRAILS[indices[j]].PATH.push_back(i);
      //       }
      //       std::cout << std::endl;
      //     }
      //   }
      // }
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
            for(int i=0; i<vertex_web[current_vertex].num_neigh; i++)
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
            std::cout << "Missioni rimanenti: " << token.MISSION.size() << std::endl;
            if (!token.MISSION.empty())
            {
              token.MISSIONS_COMPLETED[ID_ROBOT]++;
              token.TASKS_COMPLETED[ID_ROBOT] += current_mission.DSTS.size();
              std::cout << "Prendo un'altra missione" << std::endl;
              bool taken = false;
              // se il token non è vuoto prendo un altra missione
              for (int i = 0; i < token.MISSION.size(); i++)
              {
                try
                {
                  logistic_sim::Mission m1 = token.MISSION[i];
                  std::vector<uint> waypoints = { current_vertex, src_vertex };
                  for (auto dst : m1.DSTS)
                  {
                    waypoints.push_back(dst);
                  }
                  std::vector<uint> path = token_dijkstra(waypoints, token.TRAILS);
                  missions.emplace(missions.begin(), m1);
                  token.MISSION.erase(token.MISSION.begin() + i);
                  current_mission = m1;
                  taken = true;
                  token.TRAILS[ID_ROBOT].PATH = path;
                  next_vertex = token.TRAILS[ID_ROBOT].PATH[1];
                  break;
                }
                catch (const std::string& e)
                {
                  std::cout << e << std::endl;
                }
              }
            }
            else if(!go_home)
            {
              token.MISSIONS_COMPLETED[ID_ROBOT]++;
              token.TASKS_COMPLETED[ID_ROBOT] += current_mission.DSTS.size();
              std::cout << "Missioni finite, vado a casa" << std::endl;
              // next_vertex = current_vertex;
              initial_vertex = token.INIT_POS[token.INIT_POS_INDEX];
              token.INIT_POS_INDEX++;
              try
              {
                std::vector<uint> path = token_dijkstra({ current_vertex, initial_vertex }, token.TRAILS);
                token.TRAILS[ID_ROBOT].PATH = path;
                next_vertex = token.TRAILS[ID_ROBOT].PATH[1];
                go_home = true;
              }
              catch(std::string &e)
              {
                std::cout << e << std::endl;
              }
            }
            else if(!reached_home)
            {
              next_vertex = current_vertex;
              token.INIT_POS.pop_back();
              reached_home = true;
            }
            else
            {
              c_print("[ WARN]Don't know what to do!!!", yellow, P);
              next_vertex = current_vertex;
            }
            
            
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
