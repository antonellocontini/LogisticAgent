#pragma once
// virtual method specialization on cpp
namespace agent
{

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

    // initialize_node(); // robot is alive

    ros::Rate loop_rate(1);
    // routine per tornare a casa (non la usiamo pe ora)
    // ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(srvname);
    // std_srvs::Empty srv;
    // if (client.call(srv))
    // {
    //     ROS_INFO("Costmaps cleared.\n");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service move_base/clear_costmaps");
    // }e loop_rate(1);
    // ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(srvname);
    // std_srvs::Empty srv;
    // if (client.call(srv))
    // {
    //     ROS_INFO("Costmaps cleared.\n");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service move_base/clear_costmaps");
    // }
    // wait   // ros::ServiceClient client = n.serviceClient<std_srvs::Empty>(srvname);
    // std_srvs::Empty srv;
    // if (client.call(srv))
    // {
    //     ROS_INFO("Costmaps cleared.\n");
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service move_base/clear_costmaps");
    // }until nodes are ready
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
    if (!ros::param::get("/goal_reached_wait", goal_reached_wait))
    {
        //goal_reached_wait = 0.0;
        ROS_WARN("Cannot read parameter /goal_reached_wait. Using default value!");
        //ros::param::set("/goal_reached_wait", goal_reached_wait);
    }

    if (!ros::param::get("/communication_delay", communication_delay))
    {
        //communication_delay = 0.0;
        ROS_WARN("Cannot read parameter /communication_delay. Using default value!");
        //ros::param::set("/communication_delay", communication_delay);
    }

    if (!ros::param::get("/lost_message_rate", lost_message_rate))
    {
        //lost_message_rate = 0.0;
        ROS_WARN("Cannot read parameter /lost_message_rate. Using default value!");
        //ros::param::set("/lost_message_rate", lost_message_rate);
    }

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
        ROS_INFO("Goal reached ... WAITING %.2f sec", goal_reached_wait);
        ros::Duration delay(goal_reached_wait); // wait after goal is reached
        delay.sleep();
        ROS_INFO("Goal reached ... DONE");
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

            //ROS_INFO("Backup");
            backup();

            ROS_INFO("Clear costmap!");

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
{ //publicar posições
    int value = ID_ROBOT;
    if (value == -1)
    {
        value = 0;
    }
    interference = check_interference(value);
}

bool Agent::check_interference(int robot_id)
{ //verificar se os robots estao proximos

    int i;
    double dist_quad;

    if (ros::Time::now().toSec() - last_interference < 10) // seconds
        return false;                                      // false if within 10 seconds from the last one

    /* Poderei usar TEAMSIZE para afinar */
    // ID_ROBOT
    for (i = 0; i < TEAM_SIZE; i++)
    { //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
        
        //DC...
        if( i == robot_id)
            continue;

        dist_quad = (xPos[i] - xPos[robot_id]) * (xPos[i] - xPos[robot_id]) + (yPos[i] - yPos[robot_id]) * (yPos[i] - yPos[robot_id]);

        if (dist_quad <= INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE)
        { //robots are ... meter or less apart
            //          ROS_INFO("Feedback: Robots are close. INTERFERENCE! Dist_Quad = %f", dist_quad);
            last_interference = ros::Time::now().toSec();
            return true;
        }
    }
    return false;
}

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

void Agent::do_interference_behavior()
{
    ROS_INFO("Interference detected! Executing interference behavior...\n");

    // Stop the robot..
    cancelGoal();
    ROS_INFO("Robot stopped");
    ros::Duration delay(3); // seconds
    delay.sleep();
    ResendGoal = true;
}

void Agent::onGoalNotComplete()
{
    int prev_vertex = next_vertex;

    ROS_INFO("Goal not complete - From vertex %d to vertex %d\n", current_vertex, next_vertex);

    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    // Look for a random adjacent vertex different from the previous one
    int random_cnt = 0;
    while (next_vertex == prev_vertex && random_cnt++ < 10)
    {
        int num_neighs = vertex_web[current_vertex].num_neigh;
        int i = rand() % num_neighs;
        next_vertex = vertex_web[current_vertex].id_neigh[i];
        ROS_INFO("Choosing another random vertex %d\n", next_vertex);
    }

    // Look for any random vertex different from the previous one
    while (next_vertex == prev_vertex && next_vertex == current_vertex)
    {
        int i = rand() % dimension;
        next_vertex = i;
        ROS_INFO("Choosing another random vertex %d\n", next_vertex);
    }

    //Send the goal to the robot (Global Map)
    ROS_INFO("Re-Sending NEW goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    sendGoal(next_vertex); // send to move_base

    goal_complete = false;
}

bool Agent::check_neighbour_dist(int id_neighbour, double dist)
{
    double curr_dist = (xPos[ID_ROBOT] - xPos[id_neighbour])*(xPos[ID_ROBOT] - xPos[id_neighbour])
                    + (yPos[ID_ROBOT] - yPos[id_neighbour])*(yPos[ID_ROBOT] - yPos[id_neighbour]);
    
    if (curr_dist <= dist)
    {
        return false;
    }
    else
    {
        return true;
    }
}

} // namespace agent