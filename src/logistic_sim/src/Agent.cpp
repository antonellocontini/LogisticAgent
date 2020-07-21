#include "Agent.hpp"

namespace agent
{

void Agent::init(int argc, char **argv)
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

    //More than One robot (ID between 0 and 99)
    if (atoi(argv[3]) > NUM_MAX_ROBOTS || atoi(argv[3]) < -1)
    {
        ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); //max 100 robots
        return;
    }
    else
    {
        ID_ROBOT = atoi(argv[3]);
        printf("ID_ROBOT = %d\n", ID_ROBOT); //-1 for 1 robot without prefix (robot_0)
    }

    robotname = string(argv[4]);
    CAPACITY = atoi(argv[5]);
    TEAM_SIZE = atoi(argv[6]);
    mapframe = string(argv[7]);
    interactive_mode = (string(argv[8]) == "false" ? false : true);

    c_print("C and TS:", CAPACITY, TEAM_SIZE, green);

    /** D.Portugal: needed in case you "rosrun" from another folder **/
    int cmd_result = chdir(PS_path.c_str());

    mapname = string(argv[2]);
    std::string graph_file = "maps/" + mapname + "/" + mapname + ".graph";

    //Check Graph Dimension:
    dimension = GetGraphDimension(graph_file.c_str());

    //Create Structure to save the Graph Info;
    vertex_web = new vertex[dimension];

    //Get the Graph info from the Graph File
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());

    uint nedges = GetNumberEdges(vertex_web, dimension);

    printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);

    /* Define Starting Vertex/Position (Launch File Parameters) */

    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    // {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    ros::NodeHandle nh;

    // wait a random time (avoid conflicts with other robots starting at the same time...)
    double r = 3.0 * ((rand() % 1000) / 1000.0);
    ros::Duration wait(r); // seconds
    wait.sleep();

    double initial_x, initial_y;
    std::vector<double> list;
    nh.getParam("initial_pos", list);

    // list.push_back(8.1);
    // list.push_back(4.39);

    if (list.empty())
    {
        ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
        ros::shutdown();
        exit(-1);
    }

    int value = ID_ROBOT;
    if (value == -1)
    {
        value = 0;
    }

    initial_x = list[2 * value];
    initial_y = list[2 * value + 1];

    //   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
    current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    initial_vertex = current_vertex;
    //   printf("initial vertex = %d\n\n",current_vertex);

    //instantaneous idleness and last visit initialized with zeros:
    last_visit = new double[dimension];
    for (size_t i = 0; i < dimension; i++)
    {
        last_visit[i] = 0.0;

        if (i == current_vertex)
        {
            last_visit[i] = 0.1; //Avoids getting back at the initial vertex
        }
        //ROS_INFO("last_visit[%d]=%f", i, last_visit[i]);
    }

    char string1[40];
    char string2[40];

    if (ID_ROBOT == -1)
    {
        strcpy(string1, "odom");    //string = "odom"
        strcpy(string2, "cmd_vel"); //string = "cmd_vel"
        // TEAM_SIZE = 1;
    }
    else
    {
        // TODO: generalizzare nome topic per kairos
        sprintf(string1, "robot_%d/odom", ID_ROBOT);
        sprintf(string2, "robot_%d/cmd_vel", ID_ROBOT);
        // TEAM_SIZE = ID_ROBOT + 1;
    }

    /* Set up listener for global coordinates of robots */
    listener = new tf::TransformListener();

    //Cmd_vel to backup:
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(string2, 1);

    //Subscrever para obter dados de "odom" do robot corrente
    odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1, boost::bind(&Agent::odomCB, this, _1)); //size of the buffer = 1 (?)

    ros::spinOnce();

    readParams();

    token_pub = nh.advertise<logistic_sim::Token>("token", 1);

    token_sub = nh.subscribe<logistic_sim::Token>("token", 20, boost::bind(&Agent::token_callback, this, _1));

    std::cout << "mapname: " << mapname << std::endl;
    // src_vertex = map_src[mapname];
    // dsts_vertex = map_dsts[mapname];
    set_map_endpoints(nh);

    ROS_INFO_STREAM("Notyfing presence to task planner");
    // notifico la mia presenza al taskplanner
    ros::ServiceClient client = nh.serviceClient<logistic_sim::RobotReady>("robot_ready");
    logistic_sim::RobotReady srv_req;
    srv_req.request.ID_ROBOT = ID_ROBOT;
    if (!client.call(srv_req))
    {
        ROS_ERROR_STREAM("Failed to call robot_ready service!");
    }

    c_print("End of initialization, reading parameters", green, P);
}

void Agent::set_map_endpoints(ros::NodeHandle &nh)
{
  XmlRpc::XmlRpcValue src, dst;
  if (nh.getParam("/src_vertex", src))
  {
    // TODO: single source supported - move to multiple sources!
    ROS_ASSERT(src.getType() == XmlRpc::XmlRpcValue::TypeArray);
    XmlRpc::XmlRpcValue v = src[0];
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeInt);
    src_vertex = (int)v;
    ROS_INFO_STREAM("src_vertex: " << src_vertex);
  }
  else
  {
    ROS_ERROR_STREAM("Can't read param /src_vertex!!!");
    ROS_ERROR_STREAM("Currently this parameter is set by task planner launch file");
    ros::shutdown();
  }

  if (nh.getParam("/dst_vertex", dst))
  {
    ROS_ASSERT(dst.getType() == XmlRpc::XmlRpcValue::TypeArray);
    dsts_vertex.clear();
    for (int i = 0; i < dst.size(); i++)
    {
      XmlRpc::XmlRpcValue v = dst[i];
      ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeInt);
      dsts_vertex.push_back((int)v);
      ROS_INFO_STREAM("dst_vertex: " << (int)v);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Can't read param /dst_vertex!!!");
    ROS_ERROR_STREAM("Currently this parameter is set by task planner launch file");
    ros::shutdown();
  }
}

void Agent::run()
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

  while (ros::ok())
  {
    if (ResendGoal)
    {
      ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex, vertex_web[next_vertex].x,
               vertex_web[next_vertex].y);
      sendGoal(next_vertex);

      ResendGoal = false;
    }

    // if (end_simulation)
    // {
    //   return;
    // }

    loop_rate.sleep();

  }  // while ros.ok
}  // run()

void Agent::ready()
{
    char move_base[40];
    // define goal
    if (ID_ROBOT == -1)
    {
        strcpy(move_base, "move_base");
    }
    else
    {
        // sprintf(move_base, "robot_%d/move_base", ID_ROBOT);
        sprintf(move_base, "%s/move_base", robotname.c_str());
    }

    ac = new MoveBaseClient(move_base, true);

    // wait for the action server to come up
    while (!ac->waitForServer(ros::Duration(5.0)))
    {
        c_print("Waiting for the move_base action server to come up", red, P);
    }
    c_print("Connected with move_base action server", green, P);

    ros::Rate loop_rate(1);
    while (initialize)
    {
        c_print("waiting for initialization", red, P);
        ros::spinOnce();
        loop_rate.sleep();
    }

    c_print("initialization completed", magenta, P);
}

void Agent::readParams()
{
    if (!ros::param::get("/initial_positions", initial_positions))
    {
        initial_positions = "default";
        ROS_WARN("Cannot read parameter /initial_positions. Using default value '%s'!", initial_positions.c_str());
        ros::param::set("/initial_pos", initial_positions);
    }
}

void Agent::getRobotPose(int robotid, float &x, float &y, float &theta)
{

    if (listener == NULL)
    {
        ROS_ERROR("TF listener null");
        return;
    }

    std::stringstream ss;
    ss << "robot_" << robotid;
    // std::string robotname = ss.str();
    // std::string sframe = "/map"; //Patch David Portugal: Remember that the global map frame is "/map"
    std::string sframe = "/" + mapframe;
    std::string dframe;
    if (ID_ROBOT > -1)
    {
    dframe = "/" + robotname + "/base_link";
    }
    else
    {
    dframe = "/base_link";
    }

    tf::StampedTransform transform;
    
    try
    {
        listener->waitForTransform(sframe, dframe, ros::Time(0), ros::Duration(3));

        listener->lookupTransform(sframe, dframe, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot transform from %s to %s\n", sframe.c_str(), dframe.c_str());
        ROS_ERROR("%s", ex.what());
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    theta = tf::getYaw(transform.getRotation());
    // printf("Robot %d pose : %.1f %.1f \n",robotid,x,y);
}

void Agent::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{ //colocar propria posicao na tabela

    //  printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
    int idx = ID_ROBOT;

    if (ID_ROBOT <= -1)
    {
        idx = 0;
    }

    float x, y, th;
    getRobotPose(idx, x, y, th);

    xPos[idx] = x; // msg->pose.pose.position.x;
    yPos[idx] = y; // msg->pose.pose.position.y;

    //  printf("Posicao colocada em Pos[%d]\n",idx);
}

void Agent::sendGoal(int next_vertex)
{
    goal_canceled_by_user = false;

    double target_x = vertex_web[next_vertex].x,
           target_y = vertex_web[next_vertex].y;

    //Define Goal:
    move_base_msgs::MoveBaseGoal goal;
    //Send the goal to the robot (Global Map)
    geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);
    // goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.frame_id = mapframe;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = target_x;    // vertex_web[current_vertex].x;
    goal.target_pose.pose.position.y = target_y;    // vertex_web[current_vertex].y;
    goal.target_pose.pose.orientation = angle_quat; //doesn't matter really.
    ac->sendGoal(goal, boost::bind(&Agent::goalDoneCallback, this, _1, _2), boost::bind(&Agent::goalActiveCallback, this), boost::bind(&Agent::goalFeedbackCallback, this, _1));
}

void Agent::cancelGoal()
{
    goal_canceled_by_user = true;
    ac->cancelAllGoals();
}

void Agent::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{ //goal terminado (completo ou cancelado)
    //  ROS_INFO("Goal is complete (suceeded, aborted or cancelled).");
    // If the goal succeeded send a new one!
    //if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
    // If it was aborted time to back up!
    //if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;

    if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal reached ...");
        goal_complete = true;
        goal_success = true;
    }
    else
    {
        aborted_count++;
        ROS_INFO("CANCELLED or ABORTED... %d", aborted_count); //tentar voltar a enviar goal..
        if (!goal_canceled_by_user)
        {
            ROS_INFO("Goal not cancelled by the interference...");

            ROS_INFO("Backup");
            backup();

            // ROS_INFO("Clear costmap!");

            char srvname[80];

            if (ID_ROBOT <= -1)
            {
                sprintf(srvname, "/move_base/clear_costmaps");
            }
            else
            {
                sprintf(srvname, "/robot_%d/move_base/clear_costmaps", ID_ROBOT);
            }

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(srvname);
            std_srvs::Empty srv;
            if (client.call(srv))
            {
                ROS_INFO("Costmaps cleared.\n");
            }
            else
            {
                ROS_ERROR("Failed to call service move_base/clear_costmaps");
            }

            ROS_INFO("Resend Goal!");
            ResendGoal = true;
        }
    }
}

void Agent::goalActiveCallback()
{ //enquanto o robot esta a andar para o goal...
    goal_complete = false;
    //      ROS_INFO("Goal is active.");
}

void Agent::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{

}


// These are NOT the recovery behaviors of move_base
void Agent::backup()
{

    ros::Rate loop_rate(100); // 100Hz

    int backUpCounter = 0;
    while (backUpCounter <= 100)
    {

        if (backUpCounter == 0)
        {
            ROS_INFO("The wall is too close! I need to do some backing up...");
            // Move the robot back...
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = -0.1;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);
        }

        if (backUpCounter == 20)
        {
            // Turn the robot around...
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5;
            cmd_vel_pub.publish(cmd_vel);
        }

        if (backUpCounter == 100)
        {
            // Stop the robot...
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub.publish(cmd_vel);

            // ROS_INFO("Done backing up, now on with my life!");
        }

        ros::spinOnce();
        loop_rate.sleep();
        backUpCounter++;
    }
}

} // namespace agent