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
        sprintf(move_base, "robot_%d/move_base", ID_ROBOT);
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
    std::string robotname = ss.str();
    std::string sframe = "/map"; //Patch David Portugal: Remember that the global map frame is "/map"
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
    goal.target_pose.header.frame_id = "map";
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

    send_positions();

    int value = ID_ROBOT;
    if (value == -1)
    {
        value = 0;
    }
    interference = check_interference(value);
}

void Agent::send_goal_reached()
{

    int value = ID_ROBOT;
    if (value == -1)
    {
        value = 0;
    }

    // [ID,msg_type,vertex,intention,0]
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(TARGET_REACHED_MSG_TYPE);
    msg.data.push_back(current_vertex);
    //msg.data.push_back(next_vertex);
    //msg.data.push_back(0); //David Portugal: is this necessary?

    results_pub.publish(msg);
    ros::spinOnce();
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
    send_interference(); // send interference to monitor for counting

#if 1
    // Stop the robot..
    cancelGoal();
    ROS_INFO("Robot stopped");
    ros::Duration delay(3); // seconds
    delay.sleep();
    ResendGoal = true;
#else
    //get own "odom" positions...
    ros::spinOnce();

    //Waiting until conflict is solved...
    int value = ID_ROBOT;
    if (value == -1)
    {
        value = 0;
    }
    while (interference)
    {
        interference = check_interference(value);
        if (goal_complete || ResendGoal)
        {
            interference = false;
        }
    }
#endif
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

void Agent::send_positions()
{
    //Publish Position to common node:
    nav_msgs::Odometry msg;

    int idx = ID_ROBOT;

    if (ID_ROBOT <= -1)
    {
        msg.header.frame_id = "map"; //identificador do robot q publicou
        idx = 0;
    }
    else
    {
        char string[20];
        sprintf(string, "robot_%d/map", ID_ROBOT);
        msg.header.frame_id = string;
    }

    msg.pose.pose.position.x = xPos[idx]; //send odometry.x
    msg.pose.pose.position.y = yPos[idx]; //send odometry.y

    positions_pub.publish(msg);
    ros::spinOnce();
}

void Agent::receive_positions()
{
}

void Agent::positionsCB(const nav_msgs::Odometry::ConstPtr &msg)
{ //construir tabelas de posições

    //     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);

    char id[20]; //identificador do robot q enviou a msg d posição...
    strcpy(id, msg->header.frame_id.c_str());
    //int stamp = msg->header.seq;
    //     printf("robot q mandou msg = %s\n", id);

    // Build Positions Table

    if (ID_ROBOT > -1)
    {
        //verify id "XX" of robot: (string: "robot_XX/map")

        char str_idx[4];
        uint i;

        for (i = 6; i < 10; i++)
        {
            if (id[i] == '/')
            {
                str_idx[i - 6] = '\0';
                break;
            }
            else
            {
                str_idx[i - 6] = id[i];
            }
        }

        int idx = atoi(str_idx);
        //  printf("id robot q mandou msg = %d\n",idx);

        // if (idx >= TEAM_SIZE && TEAM_SIZE <= NUM_MAX_ROBOTS)
        // {
        //     //update teamsize:
        //     TEAM_SIZE = idx + 1;
        // }

        if (ID_ROBOT != idx)
        { //Ignore own positions
            xPos[idx] = msg->pose.pose.position.x;
            yPos[idx] = msg->pose.pose.position.y;
        }
        //      printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx, xPos[idx], idx, yPos[idx] );
    }

    // c_print("TEAMSIZE: ", TEAM_SIZE, red);

    receive_positions();
}

// simulates blocking send operation with delay in communication
void Agent::do_send_message(std_msgs::Int16MultiArray &msg)
{
    if (communication_delay > 0.001)
    {
        //double current_time = ros::Time::now().toSec();
        //if (current_time-last_communication_delay_time>1.0) {
        //ROS_INFO("Communication delay %.1f",communication_delay);
        ros::Duration delay(communication_delay); // seconds
        delay.sleep();
        //last_communication_delay_time = current_time;
        //}
    }
    results_pub.publish(msg);
    ros::spinOnce();
}

void Agent::send_interference()
{
    //interference: [ID,msg_type]

    int value = ID_ROBOT;
    if (value == -1)
    {
        value = 0;
    }
    printf("Send Interference: Robot %d\n", value);

    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(INTERFERENCE_MSG_TYPE);

    results_pub.publish(msg);
    ros::spinOnce();
}

 void Agent::send_resendgoal()
 {
     // TODO
 }

// DA SISTEMARE
void Agent::resultsCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
{

    std::vector<signed short>::const_iterator it = msg->data.begin();

    vresults.clear();

    for (size_t k = 0; k < msg->data.size(); k++)
    {
        vresults.push_back(*it);
        it++;
    }

    int id_sender = vresults[0];
    int msg_type = vresults[1];

    //printf(" MESSAGE FROM %d TYPE %d ...\n",id_sender, msg_type);

    // messages coming from the monitor
    if (id_sender == -1 && msg_type == INITIALIZE_MSG_TYPE)
    {
        if (initialize == true && vresults[2] == 100)
        { //"-1,msg_type,100,seq_flag" (BEGINNING)
            ROS_INFO("Let's Patrol!\n");
            double r = 1.0 * ((rand() % 1000) / 1000.0);

            //TODO if sequential start
            //r = DELTA_TIME_SEQUENTIAL_START * ID_ROBOT;

            ros::Duration wait(r); // seconds

            printf("Wait %.1f seconds (init pos:%s)\n", r, initial_positions.c_str());

            wait.sleep();
            initialize = false;
        }

#if SIMULATE_FOREVER == false
        if (initialize == false && vresults[2] == 999)
        { //"-1,msg_type,999" (END)
            ROS_INFO("The simulation is over. Let's leave");
            end_simulation = true;
        }
#endif
    }

    if (!initialize)
    {
#if 0
        // communication delay
        if(ID_ROBOT>-1){
            if ((communication_delay>0.001) && (id_sender!=ID_ROBOT)) {
                    double current_time = ros::Time::now().toSec();
                    if (current_time-last_communication_delay_time>1.0) { 
                            ROS_INFO("Communication delay %.1f",communication_delay);
                            ros::Duration delay(communication_delay); // seconds
                            delay.sleep();
                            last_communication_delay_time = current_time;
                }
            }
            bool lost_message = false;
            if ((lost_message_rate>0.0001)&& (id_sender!=ID_ROBOT)) {
                double r = (rand() % 1000)/1000.0;
                lost_message = r < lost_message_rate;
            }
            if (lost_message) {
                ROS_INFO("Lost message");
            }
        }
#endif
        receive_results();
    }

    ros::spinOnce();
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