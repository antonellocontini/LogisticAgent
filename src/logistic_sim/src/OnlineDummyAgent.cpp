#include "OnlineDummyAgent.hpp"

template <class T>
bool check_vector_equal(const std::vector<T> &v, const T &val)
{
  for (const T &x : v)
  {
    if (x != val)
    {
      return false;
    }
  }
  return true;
}

namespace onlinedummyagent
{


void OnlineDummyAgent::init(int argc, char **argv)
{
  onlineagent::OnlineAgent::init(argc, argv);
  std::string s = "patrol_robot" + std::to_string(ID_ROBOT);
}

void OnlineDummyAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
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
    token.ROBOT_WAYPOINTS.push_back(logistic_sim::Waypoints());
    token.REPAIRS_PER_ROBOT.push_back(0);

    initialize = false;
    next_vertex = current_vertex = initial_vertex;
    goal_success = true;
  }
  else if (msg->ALLOCATE)
  {
    goal_success = false;
    still = true;
    if (ID_ROBOT == TEAM_SIZE - 1)
    {
      token.ALLOCATE = false;
    }
  }
  else
  {
    // avanzamento dei robot
    token_priority_coordination(msg, token);
  }

  ros::Duration(0.03).sleep();
  token_pub.publish(token);
  ros::spinOnce();

  if (token.SHUTDOWN)
  {
    ROS_INFO_STREAM("Shutdown signal...");
    ros::shutdown();
  }
}

void OnlineDummyAgent::token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
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

  // handle goal reached event
  if (goal_success)
  {
    // check if a waypoint has been reached, for waypoint book-keeping
    if (!task_waypoints.empty())
    {
      std::list<uint> &active_waypoints = task_waypoints.front();
      if (!active_waypoints.empty() && next_vertex == active_waypoints.front())
      {
        ROS_INFO_STREAM("Reached waypoint " << next_vertex);
        active_waypoints.pop_front();
        if (active_waypoints.empty())
        {
          ROS_INFO_STREAM("Task completed!");
          task_waypoints.pop_front();
          assigned_missions.pop_front();
          ROS_INFO_STREAM("Remaining missions: " << assigned_missions.size());
        }
      }
    }

    // update travelled distance, for statistics
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

      if (can_exit && !interactive_mode)
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
    //ROS_INFO_STREAM("all reached the goal");
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

    if (token.TRAILS[ID_ROBOT].PATH.size() > 1)
    {
      next_vertex = token.TRAILS[ID_ROBOT].PATH[1];
    }
    else
    {
      c_print("[ WARN] Don't know what to do!!!", yellow, P);
      next_vertex = current_vertex;
    }

    still = false;
    c_print("before OnGoal()", magenta);
    c_print("[DEBUG]\tGoing to ", next_vertex, green, P);
    sendGoal(next_vertex);
  }

  // put in the token the current edge
  token.CURR_VERTEX[ID_ROBOT] = current_vertex;
  token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
}

}  // namespace onlinedummyagent

int main(int argc, char *argv[])
{
  onlinedummyagent::OnlineDummyAgent ODA;
  ODA.init(argc, argv);
  ROS_INFO_STREAM("@ ONLINE DCOP");
  sleep(3);
  ODA.run();
  return 0;
}