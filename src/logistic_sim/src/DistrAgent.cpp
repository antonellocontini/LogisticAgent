#include <ros/console.h>
#include "DistrAgent.hpp"

using namespace distragent;

void DistrAgent::init(int argc, char **argv)
{
  Agent::init(argc, argv);

//   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//   {
//     ros::console::notifyLoggerLevelsChanged();
//   }

  ros::NodeHandle nh;

  token_pub = nh.advertise<logistic_sim::Token>("token", 1);

  token_sub = nh.subscribe<logistic_sim::Token>("token", 20, boost::bind(&DistrAgent::token_callback, this, _1));

  std::cout << "mapname: " << mapname << std::endl;
  src_vertex = map_src[mapname];
  dsts_vertex = map_dsts[mapname];

  ROS_INFO_STREAM("Notyfing presence to task planner");
  // notifico la mia presenza al taskplanner
  ros::ServiceClient client = nh.serviceClient<logistic_sim::RobotReady>("robot_ready");
  logistic_sim::RobotReady srv_req;
  srv_req.request.ID_ROBOT = ID_ROBOT;
  if (!client.call(srv_req))
  {
      ROS_ERROR_STREAM("Failed to call robot_ready service!");
  }
}

void DistrAgent::run()
{
  // get ready
  ready();

  c_print("@ Ready!", green);

  // initially clear the costmap (to make sure the robot is not trapped):
  std_srvs::Empty srv;
  std::string mb_string;

  if (ID_ROBOT > -1)
  {
    std::ostringstream id_string;
    id_string << ID_ROBOT;
    // mb_string = "robot_" + id_string.str() + "/";
    mb_string = robotname + "/";
  }
  mb_string += "move_base/clear_costmaps";

  if (ros::service::call(mb_string.c_str(), srv))
  {
    // if (ros::service::call("move_base/clear_costmaps", srv)){
    ROS_INFO("Costmap correctly cleared before patrolling task.");
  }
  else
  {
    ROS_WARN("Was not able to clear costmap (%s) before patrolling...", mb_string.c_str());
  }

  // Asynch spinner (non-blocking)
  ros::AsyncSpinner spinner(2);  // Use n threads
  spinner.start();
  // ros::waitForShutdown();

  /* Run Algorithm */

  ros::Rate loop_rate(30);  // 0.033 seconds or 30Hz

  while (ros::ok() && !end_simulation)
  {
    if (ResendGoal)
    {
      ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex, vertex_web[next_vertex].x,
               vertex_web[next_vertex].y);
      sendGoal(next_vertex);

      ResendGoal = false;
    }

    if (end_simulation)
    {
      return;
    }

    loop_rate.sleep();

  }  // while ros.ok
}  // run()

void DistrAgent::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }
  interference = false;
}