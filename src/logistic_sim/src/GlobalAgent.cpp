#include "GlobalAgent.hpp"

using namespace globalagent;

void GlobalAgent::token_callback(const logistic_sim::TokenConstPtr& msg)
{
  // se non è per me termino
  if (msg->ID_RECEIVER != ID_ROBOT)
    return;

  logistic_sim::Token token;
  token = *msg;

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
    // controlliamo che non ci siano situazioni di stuck lungo tutti i percorsi calcolati
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
    token.INIT_POS.push_back(initial_vertex);
    // token.INIT_POS.insert(token.INIT_POS.begin(), initial_vertex);
    token.INIT_POS_INDEX = 0;
    token.MISSION_START_TIME.push_back(ros::Time::now());
    token.MISSION_CURRENT_DISTANCE.push_back(0.0f);
    token.INTERFERENCE_COUNTER.push_back(0);
    token.TOTAL_DISTANCE.push_back(0.0f);
    token.INTERFERENCE_STATUS.push_back(0);
    token.X_POS.push_back(0.0);
    token.Y_POS.push_back(0.0);
    token.GOAL_STATUS.push_back(0);

    goal_complete = true;
    path_calculated = true;
    // home_steps = token.TRAILS[ID_ROBOT].PATH.size();

    token.REACHED_HOME.push_back(false);

    initialize = false;
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
        // sono arrivato
        if (token.TRAILS[ID_ROBOT].PATH.size() == 1)
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

  ros::Duration d(0.03);
  d.sleep();
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
  globalagent::GlobalAgent GA;
  GA.init(argc, argv);
  c_print("@ SPART", green);
  sleep(3);
  GA.run();
  return 0;
}
