#include "OnlineDCOPAgent.hpp"

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

namespace onlinedcopagent
{


void OnlineDCOPAgent::init(int argc, char **argv)
{
  onlineagent::OnlineAgent::init(argc, argv);
  std::string s = "patrol_robot" + std::to_string(ID_ROBOT);
}

void OnlineDCOPAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
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

    initialize = false;
    next_vertex = current_vertex = initial_vertex;
    goal_success = true;
  }
  else if (msg->REPAIR)
  {
    if (!msg->REMOVED_EDGES.empty())
    {
      for (const logistic_sim::Edge &e : msg->REMOVED_EDGES)
      {
        int result = RemoveEdge(vertex_web, dimension, e.u, e.v);
        ROS_ERROR_STREAM_COND(result > 0, "Can't remove edge (" << e.u << "," << e.v << ")");
        ROS_INFO_STREAM_COND(result == 0, "Edge (" << e.u << "," << e.v << ") removed");
        update_graph();
      }

      // last robot removes the edges from the token
      if (msg->ID_RECEIVER == TEAM_SIZE - 1)
      {
        token.REMOVED_EDGES.clear();
        token.REPAIR = false;
        token.SINGLE_PLAN_REPAIR = true;
        // reset GOAL_STATUS vector
        token.GOAL_STATUS = std::vector<uint>(TEAM_SIZE, msg->GOAL_STATUS[0]);
      }

      // check if current path is good, if it is, no need to replan
      bool good_path = true;
      std::vector<uint> trail = msg->TRAILS[ID_ROBOT].PATH;
      trail.insert(trail.end(), msg->HOME_TRAILS[ID_ROBOT].PATH.begin(), msg->HOME_TRAILS[ID_ROBOT].PATH.end());
      for (int i = 1; i < trail.size() && good_path; i++)
      {
        uint u = trail[i - 1], v = trail[i];
        for (const logistic_sim::Edge &e : msg->REMOVED_EDGES)
        {
          if (u == e.u && v == e.v)
          {
            good_path = false;
          }
        }
      }

      if (good_path)
      {
        token.HAS_REPAIRED_PATH[ID_ROBOT] = true;
        ROS_INFO_STREAM("My path is still good, I'm keeping it!");
      }
      else
      {
        ROS_WARN_STREAM("Path unviable, need to replan");
        // delete path from token and return to current vertex
        token.TRAILS[ID_ROBOT].PATH = { current_vertex };
        token.HOME_TRAILS[ID_ROBOT].PATH.clear();
        sendGoal(current_vertex);
      }
    }
  }
  else if (msg->SINGLE_PLAN_REPAIR)
  {
    // path is not good, try repair
    if (!msg->HAS_REPAIRED_PATH[ID_ROBOT])
    {
      // merge task path with return to home path
      std::vector<logistic_sim::Path> robot_paths = token.TRAILS;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        robot_paths[i].PATH.insert(robot_paths[i].PATH.end(), token.HOME_TRAILS[i].PATH.begin(),
                                   token.HOME_TRAILS[i].PATH.end());
      }

      std::vector<uint> waypoints;
      bool failed_planning = false;
      try
      {
        std::vector<unsigned int> last_leg, first_leg;  // this will contain the path, splitted in two sections
        if (!task_waypoints.empty())
        {
          waypoints.push_back(current_vertex);
          for (uint v : task_waypoints.front())
          {
            waypoints.push_back(v);
          }
        }
        // add home vertex to waypoints
        waypoints.push_back(initial_vertex);

        // insert future tasks inside token
        for (logistic_sim::Mission &m : assigned_missions)
        {
          token.MISSION.push_back(m);
          token.NEW_MISSIONS_AVAILABLE = true;
        }

        // // calculate path
        // plan_and_update_token(waypoints, robot_paths, token, first_leg, last_leg);
        // // code after previous call execute only if plan is successful
        // token.SINGLE_PLAN_REPAIR_PROGRESS = true;
        // token.HAS_REPAIRED_PATH[ID_ROBOT] = true;
        // ROS_INFO_STREAM("Plan repaired successfully!");
        failed_planning = true;   // testing multi-agent repair
      }
      catch (std::string &e)
      {
        failed_planning = true;
      }

      if (failed_planning)
      {
        ROS_INFO_STREAM("Can't find a valid path for repair");
        token.ROBOT_WAYPOINTS[ID_ROBOT].VERTICES = waypoints;
      }
    }


    // last robot checks if there has been progress
    if (ID_ROBOT == TEAM_SIZE - 1)
    {
      bool good = check_vector_equal<uint8_t>(token.HAS_REPAIRED_PATH, true);
      if (good)
      {
        token.SINGLE_PLAN_REPAIR = false;
      }
      else
      {
        if (!token.SINGLE_PLAN_REPAIR_PROGRESS)
        {
          token.SINGLE_PLAN_REPAIR = false;
          token.MULTI_PLAN_REPAIR = true;
          ROS_WARN_STREAM("Some robots are in deadlock, going to multi-robot repair phase");
        }
      }
    }
  }
  else if (msg->MULTI_PLAN_REPAIR)
  {
    ROS_INFO_STREAM("TODO MULTI-ROBOT REPAIR PHASE");

    // here replan for the remaining tasks
  }
  else if (msg->ALLOCATE)
  {
    token_priority_alloc_plan(msg, token);
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

void OnlineDCOPAgent::token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
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
        active_waypoints.pop_front();
        if (active_waypoints.empty())
        {
          task_waypoints.pop_front();
          assigned_missions.pop_front();
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

void OnlineDCOPAgent::plan_and_update_token(const std::vector<uint> &waypoints,
                                            std::vector<logistic_sim::Path> &robot_paths, logistic_sim::Token &token,
                                            std::vector<unsigned int> &first_leg, std::vector<unsigned int> &last_leg)
{
  auto f = boost::bind(onlineagent::astar_cmp_function, min_hops_matrix, waypoints, _1, _2);
  std::vector<unsigned int> astar_result = spacetime_dijkstra(
      robot_paths, map_graph, waypoints, token.TRAILS[ID_ROBOT].PATH.size() - 1, &last_leg, &first_leg, &f);

  // update active waypoints
  if (!task_waypoints.empty())
  {
    // the last waypoint is the home, but new tasks have been inserted
    task_waypoints.back().pop_back();
  }
  std::list<uint> new_task_waypoints;
  for (int i = 1; i < waypoints.size(); i++)
  {
    new_task_waypoints.push_back(waypoints[i]);
  }
  task_waypoints.push_back(new_task_waypoints);

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
}

void OnlineDCOPAgent::token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token)
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
    plan_and_update_token(waypoints, robot_paths, token, first_leg, last_leg);
    // add mission to local list
    assigned_missions.push_back(m);

    // update token statistics
    token.MISSIONS_COMPLETED[ID_ROBOT]++;
    token.TASKS_COMPLETED[ID_ROBOT] += m.DEMANDS.size();

    // remove mission from token
    token.MISSION.erase(token.MISSION.begin());

    // booleans in ros msgs becomes uint8_t
    // task allocated successfully, reset FAILED_ALLOCATION vector
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


}  // namespace onlinedcopagent

int main(int argc, char *argv[])
{
  onlinedcopagent::OnlineDCOPAgent ODA;
  ODA.init(argc, argv);
  ROS_INFO_STREAM("@ ONLINE DCOP");
  sleep(3);
  ODA.run();
  return 0;
}