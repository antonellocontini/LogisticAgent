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
  else
  {
    ROS_INFO_STREAM("TODO DCOP TOKEN");
  }

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