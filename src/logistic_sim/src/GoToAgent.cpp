#include <limits>
#include "GoToAgent.hpp"

namespace gotoagent
{
void GoToAgent::init(int argc, char **argv)
{
  /*
            argv[0]=/.../patrolling_sim/bin/GBS
            argv[1]=__name:=XXXXXX
            argv[2]=grid
            argv[3]=ID_ROBOT
            argv[4]=nome robot
            argv[5]=CAPACITY
            argv[6]=TEAMSIZE
            argv[7]=MAP_FRAME
            argv[8]=INTERACTIVE_MODE
        */

  srand(time(NULL));

  // More than One robot (ID between 0 and 99)
  if (atoi(argv[3]) > NUM_MAX_ROBOTS || atoi(argv[3]) < -1)
  {
    ROS_INFO("The Robot's ID must be an integer number between 0 an 99");  // max 100 robots
    return;
  }
  else
  {
    ID_ROBOT = atoi(argv[3]);
  }

  robotname = string(argv[4]);
  CAPACITY = atoi(argv[5]);
  TEAM_SIZE = atoi(argv[6]);
  mapframe = string(argv[7]);
  interactive_mode = (string(argv[8]) == "false" ? false : true);

  /** D.Portugal: needed in case you "rosrun" from another folder **/
  int cmd_result = chdir(PS_path.c_str());

  mapname = string(argv[2]);
  std::string graph_file = "maps/" + mapname + "/" + mapname + ".graph";

  // Check Graph Dimension:
  // dimension = GetGraphDimension(graph_file.c_str());

  // Create Structure to save the Graph Info;
  // vertex_web = new vertex[dimension];

  // Get the Graph info from the Graph File
  // GetGraphInfo(vertex_web, dimension, graph_file.c_str());

  // uint nedges = GetNumberEdges(vertex_web, dimension);

  // printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);

  /* Define Starting Vertex/Position (Launch File Parameters) */

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //     ros::console::notifyLoggerLevelsChanged();
  // }

  ros::init(argc, argv, "patrol_agent");  // will be replaced by __name:=XXXXXX
  ros::NodeHandle nh;

  // wait a random time (avoid conflicts with other robots starting at the same time...)
  double r = 3.0 * ((rand() % 1000) / 1000.0);
  ros::Duration wait(r);  // seconds
  wait.sleep();

  // double initial_x, initial_y;
  // std::vector<double> list;
  // nh.getParam("initial_pos", list);

  // // list.push_back(8.1);
  // // list.push_back(4.39);

  // if (list.empty())
  // {
  //   ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
  //   ros::shutdown();
  //   exit(-1);
  // }
  // ROS_INFO_STREAM("initial_pos list read correctly");

  // int value = ID_ROBOT;
  // if (value == -1)
  // {
  //   value = 0;
  // }

  // initial_x = list[2 * value];
  // initial_y = list[2 * value + 1];

  // current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
  // initial_vertex = current_vertex;
  //   printf("initial vertex = %d\n\n",current_vertex);

  // instantaneous idleness and last visit initialized with zeros:
  // last_visit = new double[dimension];
  // for (size_t i = 0; i < dimension; i++)
  // {
  //   last_visit[i] = 0.0;

  //   if (i == current_vertex)
  //   {
  //     last_visit[i] = 0.1;  // Avoids getting back at the initial vertex
  //   }
  //   // ROS_INFO("last_visit[%d]=%f", i, last_visit[i]);
  // }

  char string1[40];
  char string2[40];

  sprintf(string1, "%s/odom", robotname.c_str());
  sprintf(string2, "%s/cmd_vel", robotname.c_str());
  // sprintf(string1, "robot_%d/odom", ID_ROBOT);
  // sprintf(string2, "robot_%d/cmd_vel", ID_ROBOT);
  // TEAM_SIZE = ID_ROBOT + 1;

  /* Set up listener for global coordinates of robots */
  listener = new tf::TransformListener();

  // Cmd_vel to backup:
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(string2, 1);

  // Subscrever para obter dados de "odom" do robot corrente
  odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1,
                                              boost::bind(&Agent::odomCB, this, _1));  // size of the buffer = 1 (?)

  ros::spinOnce();

  // readParams();

  // token_pub = nh.advertise<logistic_sim::Token>("token", 1);

  // token_sub = nh.subscribe<logistic_sim::Token>("token", 20, boost::bind(&Agent::token_callback, this, _1));

  std::cout << "mapname: " << mapname << std::endl;
  // src_vertex = map_src[mapname];
  // dsts_vertex = map_dsts[mapname];
  // set_map_endpoints(nh);

  // ROS_INFO_STREAM("Notyfing presence to task planner");
  // // notifico la mia presenza al taskplanner
  // ros::ServiceClient client = nh.serviceClient<logistic_sim::RobotReady>("robot_ready");
  // logistic_sim::RobotReady srv_req;
  // srv_req.request.ID_ROBOT = ID_ROBOT;
  // if (!client.call(srv_req))
  // {
  //   ROS_ERROR_STREAM("Failed to call robot_ready service!");
  // }

  // c_print("End of initialization, reading parameters", green, P);
  ROS_INFO_STREAM("Advertising goto_pos service to " + ros::this_node::getName() + "/goto_pos");
  goto_pos_service = nh.advertiseService(ros::this_node::getName() + "/goto_pos", &GoToAgent::goto_pos, this);
  if (!goto_pos_service)
  {
    ROS_ERROR_STREAM("Can't create goto_pos service");
  }
  else
  {
    ROS_INFO_STREAM("goto_pos service advertised successfully");
  }

  ROS_INFO_STREAM("Advertising cancel_goto service to " + ros::this_node::getName() + "/cancel_goto");
  cancel_goto_service = nh.advertiseService(ros::this_node::getName() + "/cancel_goto", &GoToAgent::cancel_goto, this);
  if (!cancel_goto_service)
  {
    ROS_ERROR_STREAM("Can't create cancel_goto service");
  }
  else
  {
    ROS_INFO_STREAM("cancel_goto service advertised successfully");
  }
}

void GoToAgent::run()
{
  // get ready
  char move_base[40];
  sprintf(move_base, "%s/move_base", robotname.c_str());

  ac = new MoveBaseClient(move_base, true);

  ROS_INFO_STREAM("Connecting to move_base action server at " << move_base);
  // wait for the action server to come up
  while (!ac->waitForServer(ros::Duration(5.0)))
  {
    ROS_WARN_STREAM("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("Connected with move_base action server");

  ROS_INFO_STREAM("initialization completed");

  // initially clear the costmap (to make sure the robot is not trapped):
  std_srvs::Empty srv;
  std::string mb_string;

  if (ID_ROBOT > -1)
  {
    std::ostringstream id_string;
    id_string << ID_ROBOT;
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
  // ros::AsyncSpinner spinner(2);  // Use n threads
  // spinner.start();
  // ros::waitForShutdown();

  /* Run Algorithm */

  ros::spin();
}

void GoToAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
}

bool GoToAgent::goto_pos(logistic_sim::GoToPos::Request &msg, logistic_sim::GoToPos::Response &res)
{
  ROS_INFO("Sending goal to position (%f,%f) orientation %f",msg.x, msg.y, msg.theta);
  // Define Goal:
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = mapframe;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = msg.x;
  goal.target_pose.pose.position.y = msg.y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(msg.theta);
  // Send the goal to the robot (Global Map)
  ac->sendGoal(goal, boost::bind(&Agent::goalDoneCallback, this, _1, _2), boost::bind(&Agent::goalActiveCallback, this),
               boost::bind(&Agent::goalFeedbackCallback, this, _1));
  res.response = true;
  return true;
}

bool GoToAgent::cancel_goto(logistic_sim::CancelGoTo::Request &msg, logistic_sim::CancelGoTo::Response &res)
{
  goal_canceled_by_user = true;
  ac->cancelGoal();
  res.response = true;
  return true;
}

void GoToAgent::goalDoneCallback(const actionlib::SimpleClientGoalState &state,
                                 const move_base_msgs::MoveBaseResultConstPtr &result)
{
  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Goal reached ...");
    goal_complete = true;
    goal_success = true;
  }
  else
  {
    aborted_count++;
    ROS_INFO("CANCELLED or ABORTED...");  // tentar voltar a enviar goal..
    if (!goal_canceled_by_user)
    {
      ROS_WARN("Goal not cancelled by user...");
    }
    else
    {
      ROS_INFO("Goal cancelled by user");
      goal_canceled_by_user = false;
    }
  }
}

}  // namespace onlineagent

int main(int argc, char *argv[])
{
  gotoagent::GoToAgent GA;
  GA.init(argc, argv);
  if (ros::ok())
  {
    c_print("@ GOTO", green);
    sleep(3);
    GA.run();
  }
  return 0;
}