// specialized method (virtual)
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

    c_print("End of initialization, reading parameters", green, P);
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

} // namespace agent