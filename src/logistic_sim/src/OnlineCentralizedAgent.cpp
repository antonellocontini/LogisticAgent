#include <limits>
#include "OnlineCentralizedAgent.hpp"

namespace onlinecentralizedagent
{
void OnlineCentralizedAgent::init(int argc, char **argv)
{
  Agent::init(argc, argv);
}

void OnlineCentralizedAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
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
  else
  {
    // avanzamento dei robot
    token_coordination(msg, token);
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


void OnlineCentralizedAgent::token_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
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
    // se ci sono nuove missioni disponibili bisogna allocarle
    // IMPORTANTE: è essenziale che i robot siano sincronizzati
    // sul goal altrimenti la pianificazione si baserà
    // su dei percorsi sfasati
    bool first_to_see_equal = false;
    status = msg->GOAL_STATUS[0];
    for (int i = 1; i < TEAM_SIZE; i++)
    {
      if (msg->GOAL_STATUS[i] != status)
        first_to_see_equal = true;
    }

    if (msg->NEW_MISSIONS_AVAILABLE && first_to_see_equal)
    {
      token.SYNCED_ROBOTS = true;
      token.ID_RECEIVER = TASK_PLANNER_ID;
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

  // metto nel token quale arco sto occupando
  token.CURR_VERTEX[ID_ROBOT] = current_vertex;
  token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
}

}  // namespace onlinecentralizedagent

int main(int argc, char *argv[])
{
  onlinecentralizedagent::OnlineCentralizedAgent OCA;
  OCA.init(argc, argv);
  c_print("@ ONLINE", green);
  sleep(3);
  OCA.run();
  return 0;
}