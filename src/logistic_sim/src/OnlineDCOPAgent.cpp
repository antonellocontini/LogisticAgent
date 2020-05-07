#include "OnlineDCOPAgent.hpp"

namespace onlinedcopagent
{
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

    initialize = false;
    next_vertex = current_vertex = initial_vertex;
    goal_success = true;
  }
  else if (!msg->REMOVED_EDGES.empty())
  {
    ROS_INFO_STREAM("TODO HANDLE EDGE REMOVAL");
    for (const logistic_sim::Edge &e : msg->REMOVED_EDGES)
    {
      int result = RemoveEdge(vertex_web, dimension, e.u, e.v);
      ROS_ERROR_STREAM_COND(result > 0, "Can't remove edge (" << e.u << "," << e.v << ")");
      ROS_INFO_STREAM_COND(result == 0, "Edge (" << e.u << "," << e.v << ") removed");
    }

    // last robot removes the edges from the token
    if (msg->ID_RECEIVER == TEAM_SIZE - 1)
    {
      token.REMOVED_EDGES.clear();
    }
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
    // ROS_INFO_STREAM("TODO DCOP TOKEN");
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