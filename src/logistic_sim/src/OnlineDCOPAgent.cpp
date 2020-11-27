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
    token.TOTAL_STEPS.push_back(0);
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
  else if (msg->REPAIR)
  {
    if (!msg->ADDED_VERTEX.empty())
    {
      for (const logistic_sim::Vertex &v : msg->ADDED_VERTEX)
      {
        ROS_INFO_STREAM("Adding vertex to vertex_web");
        vertex_web = AddVertexCoord(vertex_web, dimension, v.id, v.x, v.y);
        dimension = dimension + 1;
        //ROS_INFO_STREAM("DIMENSION: " << dimension);
      }
      update_graph();
    }

    if (!msg->REMOVED_EDGES.empty() || !msg->ADDED_EDGES.empty())
    {
      for (const logistic_sim::Edge &e : msg->REMOVED_EDGES)
      {
        int result = RemoveEdge(vertex_web, dimension, e.u, e.v);
        ROS_ERROR_STREAM_COND(result > 0, "Can't remove edge (" << e.u << "," << e.v << ")");
        ROS_INFO_STREAM_COND(result == 0, "Edge (" << e.u << "," << e.v << ") removed");
        // update_graph();
      }

      for (const logistic_sim::Edge &e : msg->ADDED_EDGES)
      {
        int result = AddEdge(vertex_web, dimension, e.u, e.v, 1);
        ROS_ERROR_STREAM_COND(result > 0, "Can't add edge (" << e.u << "," << e.v << ")");
        ROS_INFO_STREAM_COND(result == 0, "Edge (" << e.u << "," << e.v << ") added");
      }
      update_graph();
    }

    if (!msg->REMOVED_VERTEX.empty())
    {
      for (const logistic_sim::Vertex &v : msg->REMOVED_VERTEX)
      {
        ROS_INFO_STREAM("Removing vertex from vertex_web");
        vertex_web = RemoveVertexCoord(vertex_web, dimension, v.id);
        dimension = dimension - 1;
        // update_graph();
      }
      update_graph();
    }
    

    // last robot removes the edges from the token
    if (msg->ID_RECEIVER == TEAM_SIZE - 1)
    {
      token.REMOVED_EDGES.clear();
      token.ADDED_EDGES.clear();
      token.REMOVED_VERTEX.clear();
      token.ADDED_VERTEX.clear();
      token.REPAIR = false;
      token.SINGLE_PLAN_REPAIR = true;
      // reset GOAL_STATUS vector
      token.GOAL_STATUS = std::vector<uint>(TEAM_SIZE, msg->GOAL_STATUS[0]);
    }

    // if a new home has been assigned, take it
    if (msg->NEW_HOMES[ID_ROBOT] != -1)
    {
      initial_vertex = msg->NEW_HOMES[ID_ROBOT];
      token.NEW_HOMES[ID_ROBOT] = -1;
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
        if ((u == e.u && v == e.v) || 
              (u == e.v && v == e.u))
        {
          good_path = false;
        }
      }
    }

    // test
    good_path = false;

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
        waypoints.push_back(token.TRAILS[ID_ROBOT].PATH.back());
        if (!task_waypoints.empty())
        {
          for (uint v : task_waypoints.front())
          {
            if (v == src_vertex || std::find(dsts_vertex.begin(), dsts_vertex.end(), v) != dsts_vertex.end())
            {
              waypoints.push_back(v);
            }
          }
        }
        // add home vertex to waypoints
        waypoints.push_back(initial_vertex);

        // insert future tasks inside token
        auto it = assigned_missions.begin();
        if (it != assigned_missions.end())
        {
          it++;
          for (; it != assigned_missions.end(); it++)
          {
            token.MISSION.push_back(*it);
            token.NEW_MISSIONS_AVAILABLE = true;
            // update token statistics
            token.MISSIONS_COMPLETED[ID_ROBOT]--;
            token.TASKS_COMPLETED[ID_ROBOT] -= (*it).DEMANDS.size();
            ROS_INFO_STREAM("mission removed: " << (*it).DEMANDS.size() << " tasks removed!");
            ROS_INFO_STREAM("completed missions: " << token.MISSIONS_COMPLETED[ID_ROBOT]);
            ROS_INFO_STREAM("completed tasks: " << token.TASKS_COMPLETED[ID_ROBOT]);
          }
        }
        it = assigned_missions.begin();
        it++;
        assigned_missions.erase(it, assigned_missions.end());

        // calculate path
        // if (ID_ROBOT % 2 == 0)
        // {
          allocate_memory();
          plan_and_update_token(waypoints, robot_paths, token, first_leg, last_leg);
          if (token.TRAILS[ID_ROBOT].PATH.size() == 1 && !token.HOME_TRAILS[ID_ROBOT].PATH.empty())
          {
            token.TRAILS[ID_ROBOT].PATH.push_back(token.HOME_TRAILS[ID_ROBOT].PATH.front());
            token.HOME_TRAILS[ID_ROBOT].PATH.erase(token.HOME_TRAILS[ID_ROBOT].PATH.begin());
          }
          // code after previous call execute only if plan is successful
          // update active waypoints
          task_waypoints.clear();
          std::list<uint> new_task_waypoints;
          // the first waypoint is the current vertex, it must not be inserted
          for (int i = 1; i < waypoints.size(); i++)
          {
            new_task_waypoints.push_back(waypoints[i]);
          }
          task_waypoints.push_back(new_task_waypoints);
          token.SINGLE_PLAN_REPAIR_PROGRESS = true;
          token.HAS_REPAIRED_PATH[ID_ROBOT] = true;
          token.REPAIRS_PER_ROBOT[ID_ROBOT]++;
          ROS_INFO_STREAM("Plan repaired successfully!");
        // }
        // else
        // {
        //   failed_planning = true;   // testing multi-agent repair
        // }
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
        token.SUCCESSFULL_SA_REPAIR++;
        token.OBSTACLE_EVENTS++;
      }
      else
      {
        if (!token.SINGLE_PLAN_REPAIR_PROGRESS)
        {
          token.SINGLE_PLAN_REPAIR = false;
          token.MULTI_PLAN_REPAIR = true;
          ROS_WARN_STREAM("Some robots are in deadlock, going to multi-robot repair phase");
        }
        else
        {
          token.SINGLE_PLAN_REPAIR_PROGRESS = false;
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
    token.TOTAL_STEPS[ID_ROBOT] += 1;

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

    if (msg->NEED_REPAIR && (first_to_see_equal || TEAM_SIZE == 1))
    {
      // start the plan repair phase
      token.NEED_REPAIR = false;
      token.REPAIR = true;
      token.ID_RECEIVER = 0;
    }
    else if (msg->NEW_MISSIONS_AVAILABLE && (first_to_see_equal || TEAM_SIZE == 1))
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
        c_print("[ WARN] Don't know what to do!!!", yellow, P);
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
    // update active waypoints
    if (!task_waypoints.empty())
    {
      // the last waypoint is the home, but new tasks have been inserted
      task_waypoints.back().pop_back();
    }
    std::list<uint> new_task_waypoints;
    // the first waypoint is the current vertex, it must not be inserted
    for (int i = 1; i < waypoints.size(); i++)
    {
      new_task_waypoints.push_back(waypoints[i]);
    }
    task_waypoints.push_back(new_task_waypoints);
    // add mission to local list
    assigned_missions.push_back(m);

    // update token statistics
    token.MISSIONS_COMPLETED[ID_ROBOT]++;
    token.TASKS_COMPLETED[ID_ROBOT] += m.DEMANDS.size();
    ROS_INFO_STREAM("mission added: " << m.DEMANDS.size() << " new tasks added!");
    ROS_INFO_STREAM("completed missions: " << token.MISSIONS_COMPLETED[ID_ROBOT]);
    ROS_INFO_STREAM("completed tasks: " << token.TASKS_COMPLETED[ID_ROBOT]);

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


uint OnlineDCOPAgent::calculate_h_value(const single_agent_node &n, const std::vector<unsigned int> &waypoints,
                                        const std::vector<std::vector<unsigned int>> &min_hops_matrix)
{
  uint result = 0;
  uint prev_v = n.vertex;
  for (uint i = n.waypoint; i < waypoints.size(); i++)
  {
    result += min_hops_matrix[prev_v][waypoints[i]];
    prev_v = waypoints[i];
  }
  return result;
  // return min_hops_matrix[n.vertex][waypoints[n.waypoint]];
}

std::vector<unsigned int> OnlineDCOPAgent::spacetime_astar_dyn_mem(
    const std::vector<logistic_sim::Path> &other_paths, const std::vector<std::vector<unsigned int>> &graph,
    const std::vector<unsigned int> &waypoints, const std::vector<std::vector<unsigned int>> &min_hops_matrix,
    int start_time, std::vector<unsigned int> *last_leg, std::vector<unsigned int> *first_leg)
{
  ROS_INFO_STREAM("SPACETIME DIJKSTRA --- WAYPOINTS:");
  for (int i = 0; i < waypoints.size(); i++)
  {
    ROS_INFO_STREAM(" " << waypoints[i]);
  }

  std::set<single_agent_node> open;
  std::unordered_set<single_agent_node, sa_node_hash> visited;
  std::unordered_map<single_agent_node, single_agent_node, sa_node_hash> prev;

  single_agent_node source;
  source.vertex = waypoints.front();
  source.timestep = start_time;
  source.waypoint = 1;
  source.f = calculate_h_value(source, waypoints, min_hops_matrix);

  open.insert(source);

  while (!open.empty())
  {
    const single_agent_node &current_node = *open.begin();
    visited.insert(current_node);

    single_agent_node next_node;
    next_node.timestep = current_node.timestep + 1;
    next_node.waypoint = current_node.waypoint;

    if (current_node.vertex == waypoints[current_node.waypoint])
    {
      // check if current node is last goal
      if (current_node.waypoint + 1 == waypoints.size())
      {
        bool good = true;
        for (int i = 0; i < TEAM_SIZE && good; i++)
        {
          if (i != ID_ROBOT && other_paths[i].PATH.size() > current_node.timestep + 1)
          {
            for (int j = current_node.timestep; j < other_paths[i].PATH.size(); j++)
            {
              if (other_paths[i].PATH[j] == current_node.vertex)
              {
                good = false;
                break;
              }
            }
          }
        }

        // current node is final node, return path
        if (good)
        {
          std::list<unsigned int> path, first_leg_ls, last_leg_ls;
          bool is_last_leg = true;

          single_agent_node temp = current_node;
          auto it = prev.end();
          while (true)
          {
            if (temp.vertex == waypoints[temp.waypoint - 1])
            {
              is_last_leg = false;
            }

            path.push_front(temp.vertex);
            if (is_last_leg)
            {
              last_leg_ls.push_front(temp.vertex);
            }
            else
            {
              first_leg_ls.push_front(temp.vertex);
            }

            it = prev.find(temp);
            if (it != prev.end())
            {
              temp = it->second;
            }
            else
            {
              if (first_leg != nullptr && last_leg != nullptr)
              {
                first_leg->clear();
                last_leg->clear();
                for (uint v : first_leg_ls)
                {
                  first_leg->push_back(v);
                }
                for (uint v : last_leg_ls)
                {
                  last_leg->push_back(v);
                }
              }
              std::vector<uint> result;
              for (uint v : path)
              {
                result.push_back(v);
              }
              return result;
            }
          }
        }
      }
      else
      {
        next_node.waypoint = current_node.waypoint + 1;
      }
    }

    // attesa sul posto
    next_node.vertex = current_node.vertex;
    next_node.f = calculate_h_value(next_node, waypoints, min_hops_matrix);
    if (visited.find(next_node) == visited.end())
    {
      bool good = true;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        if (i != ID_ROBOT)
        {
          if (next_node.timestep < other_paths[i].PATH.size())
          {
            if (other_paths[i].PATH[next_node.timestep] == current_node.vertex)
            {
              // std::cout << "can't stand in " << u << " at time " << time << std::endl;
              good = false;
              break;
            }
          }
          else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
          {
            if (other_paths[i].PATH.back() == current_node.vertex)
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
        open.insert(next_node);
      }
    }

    // considero i vertici vicini
    for (uint v : graph[current_node.vertex])
    {
      next_node.vertex = v;
      next_node.f = calculate_h_value(next_node, waypoints, min_hops_matrix);

      if (visited.find(next_node) == visited.end())
      {
        bool good = true;
        for (int i = 0; i < TEAM_SIZE; i++)
        {
          if (i != ID_ROBOT)
          {
            if (next_node.timestep < other_paths[i].PATH.size())
            {
              if (other_paths[i].PATH[next_node.timestep] == next_node.vertex)
              {
                // std::cout << "can't go in " << v << " at time " << time << std::endl;
                good = false;
                break;
              }
              if (other_paths[i].PATH[next_node.timestep] == current_node.vertex && other_paths[i].PATH[current_node.timestep] == next_node.vertex)
              {
                // std::cout << "can't go in " << v << " at time " << time << std::endl;
                good = false;
                break;
              }
            }
            else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
            {
              if (other_paths[i].PATH.back() == next_node.vertex)
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
          open.insert(next_node);
        }
      }
    }
  }

  ROS_WARN_STREAM("Can't find path!!!");
  throw std::string("Can't find path!!!");
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