/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
*********************************************************************/

#include "PatrolAgent.hpp"

namespace patrolagent
{
void PatrolAgent::ready()
{
  char move_string[40];

  /* Define Goal */
  if (ID_ROBOT == -1)
  {
    strcpy(move_string, "move_base"); // string = "move_base
  }
  else
  {
    // sprintf(move_string, "robot_%d/move_base", ID_ROBOT);
    sprintf(move_string, "%s/move_base", robotname.c_str());
  }

  ac = new MoveBaseClient(move_string, true);

  // wait for the action server to come up
  while (!ac->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ROS_INFO("Connected with move_base action server");

  // initialize_node(); // announce that agent is alive

  ros::Rate loop_rate(1); // 1 sec

  /* Wait until all nodes are ready.. */
  // while (initialize)
  // {
    send_initialize_msg();
    ros::spinOnce();
    loop_rate.sleep();
  // }
}

void PatrolAgent::readParams()
{
  if (!ros::param::get("/goal_reached_wait", goal_reached_wait))
  {
    goal_reached_wait = 0.0;
    ROS_WARN("Cannot read parameter /goal_reached_wait. Using default value!");
    ros::param::set("/goal_reached_wait", goal_reached_wait);
  }

  if (!ros::param::get("/communication_delay", communication_delay))
  {
    communication_delay = 0.0;
    ROS_WARN("Cannot read parameter /communication_delay. Using default value!");
    ros::param::set("/communication_delay", communication_delay);
  }

  if (!ros::param::get("/lost_message_rate", lost_message_rate))
  {
    lost_message_rate = 0.0;
    ROS_WARN("Cannot read parameter /lost_message_rate. Using default value!");
    ros::param::set("/lost_message_rate", lost_message_rate);
  }

  // if (!ros::param::get("/initial_positions", initial_positions))
  // {
  //   initial_positions = "default";
  //   ROS_WARN("Cannot read parameter /initial_positions. Using default value '%s'!", initial_positions.c_str());
  //   ros::param::set("/initial_pos", initial_positions);
  // }
}

////////////////////////////////////////////////////////////////////////////////

void PatrolAgent::send_initialize_msg()
{ //ID,msg_type,1

  ROS_INFO("Initialize Node: Robot %d", ID_ROBOT);

  std_msgs::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(ID_ROBOT);
  msg.data.push_back(INITIALIZE_MSG_TYPE);
  msg.data.push_back(1); // Robot initialized

  int count = 0;

  //ATENÇÃO ao PUBLICADOR!
  ros::Rate loop_rate(1); //meio segundo

  //while (true){ //send activation msg forever
  //results_pub.publish(msg);
  //ROS_INFO("publiquei msg: %s\n", msg.data.c_str());
  //ros::spinOnce();
  do_send_message(msg);
  loop_rate.sleep();
  count++;
  //}
}

// void PatrolAgent::initialize_node()
// { // ID,msg_type,1

//   int value = ID_ROBOT;
//   if (value == -1)
//   {
//     value = 0;
//   }
//   ROS_INFO("Initialize Node: Robot %d", value);

//   std_msgs::Int16MultiArray msg;
//   msg.data.clear();
//   msg.data.push_back(value);
//   msg.data.push_back(INITIALIZE_MSG_TYPE);
//   msg.data.push_back(1); // Robot initialized

//   int count = 0;

//   // ATENÇÃO ao PUBLICADOR!
//   ros::Rate loop_rate(0.5); // meio segundo

//   while (count < 3)
//   { // send activation msg 3times
//     results_pub.publish(msg);
//     // ROS_INFO("publiquei msg: %s\n", msg.data.c_str());
//     ros::spinOnce();
//     loop_rate.sleep();
//     count++;
//   }
// }

void PatrolAgent::getRobotPose(int robotid, float &x, float &y, float &theta)
{
  if (listener == NULL)
  {
    ROS_ERROR("TF listener null");
    return;
  }

  std::stringstream ss;
  ss << "robot_" << robotid;
  std::string robotname = ss.str();
  std::string sframe = "/map"; // Patch David Portugal: Remember that the global map frame is "/map"
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

void PatrolAgent::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{ // colocar propria posicao na tabela

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

// void PatrolAgent::poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// { //colocar propria posicao na tabela

//   //  printf("Colocar Propria posição na tabela, ID_ROBOT = %d\n",ID_ROBOT);
//   int idx = ID_ROBOT;

//   if (ID_ROBOT <= -1)
//   {
//     idx = 0;
//   }

//   //float x,y,th;
//   //getRobotPose(idx,x,y,th);

//   xPos[idx] = msg->pose.pose.position.x;
//   yPos[idx] = msg->pose.pose.position.y;
//   thetaPos[idx] = tf::getYaw(msg->pose.pose.orientation);
//   //printf(" POSITION RECEIVED x:%f y:%f \n",xPos[idx],yPos[idx]);
//   //  printf("Posicao colocada em Pos[%d]\n",idx);
// }

void PatrolAgent::sendGoal(int next_vertex)
{
  goal_canceled_by_user = false;

  double target_x = vertex_web[next_vertex].x, target_y = vertex_web[next_vertex].y;

  // Define Goal:
  move_base_msgs::MoveBaseGoal goal;
  // Send the goal to the robot (Global Map)
  geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(0.0);
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = target_x;    // vertex_web[current_vertex].x;
  goal.target_pose.pose.position.y = target_y;    // vertex_web[current_vertex].y;
  goal.target_pose.pose.orientation = angle_quat; // doesn't matter really.
  ac->sendGoal(goal, boost::bind(&PatrolAgent::goalDoneCallback, this, _1, _2),
               boost::bind(&PatrolAgent::goalActiveCallback, this),
               boost::bind(&PatrolAgent::goalFeedbackCallback, this, _1));
}

void PatrolAgent::cancelGoal()
{
  goal_canceled_by_user = true;
  ac->cancelAllGoals();
}

void PatrolAgent::goalDoneCallback(const actionlib::SimpleClientGoalState &state,
                                   const move_base_msgs::MoveBaseResultConstPtr &result)
{ // goal terminado (completo ou cancelado)
  //  ROS_INFO("Goal is complete (suceeded, aborted or cancelled).");
  // If the goal succeeded send a new one!
  // if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) sendNewGoal = true;
  // If it was aborted time to back up!
  // if(state.state_ == actionlib::SimpleClientGoalState::ABORTED) needToBackUp = true;

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Goal reached ... WAITING %.2f sec", goal_reached_wait);
    // ros::Duration delay(goal_reached_wait); // wait after goal is reached
    // delay.sleep();
    ROS_INFO("Goal reached ... DONE");
    goal_complete = true;
  }
  else
  {
    aborted_count++;
    ROS_INFO("CANCELLED or ABORTED... %d", aborted_count); // tentar voltar a enviar goal..
    if (!goal_canceled_by_user)
    {
      ROS_INFO("Goal not cancelled by the interference...");

      // ROS_INFO("Backup");

      // backup();
      /* 
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
      } */

      ROS_INFO("Resend Goal!");
      ResendGoal = true;
    }
  }
}

void PatrolAgent::goalActiveCallback()
{ // enquanto o robot esta a andar para o goal...
  goal_complete = false;
  //      ROS_INFO("Goal is active.");
}

void PatrolAgent::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{ // publicar posições

  send_positions();

  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }
  interference = check_interference(value);
}

void PatrolAgent::send_task_reached()
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
  msg.data.push_back(TASK_REACHED_MSG_TYPE);
  msg.data.push_back(1);
  msg.data.push_back(mission.front().item);
  cout << "### SEND path distance:" << mission.front().path_distance << "\n";
  msg.data.push_back(mission.front().path_distance);
  msg.data.push_back(interference_cnt);
  msg.data.push_back(resend_goal_count);
  // msg.data.push_back(next_vertex);
  // msg.data.push_back(0); //David Portugal: is this necessary?

  // results_pub.publish(msg);
  ros::spinOnce();
}

void PatrolAgent::send_goal_reached()
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

  do_send_message(msg);
  // results_pub.publish(msg);
  // ros::spinOnce();
}

bool PatrolAgent::check_interference(int robot_id)
{ // verificar se os robots estao proximos

  int i;
  double dist_quad;

  if (ros::Time::now().toSec() - last_interference < 10) // seconds
    return false;                                        // false if within 10 seconds from the last one

  // /* Poderei usar TEAMSIZE para afinar */ <<<<<<<<<<<<<<<
  for (i = 0; i <= robot_id; i++)
  { // percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)

    dist_quad = (xPos[i] - xPos[robot_id]) * (xPos[i] - xPos[robot_id]) +
                (yPos[i] - yPos[robot_id]) * (yPos[i] - yPos[robot_id]);

    if (dist_quad <= INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE)
    {
      // robots are ... meter or less apart
      //          ROS_INFO("Feedback: Robots are close. INTERFERENCE! Dist_Quad = %f", dist_quad);
      last_interference = ros::Time::now().toSec();
      return true;
    }
  }
  return false;
}

// void PatrolAgent::backup()
// {
//   ros::Rate loop_rate(100); // 100Hz

//   int backUpCounter = 0;
//   while (backUpCounter <= 100)
//   {
//     if (backUpCounter == 0)
//     {
//       ROS_INFO("The wall is too close! I need to do some backing up...");
//       // Move the robot back...
//       geometry_msgs::Twist cmd_vel;
//       cmd_vel.linear.x = -0.1;
//       cmd_vel.angular.z = 0.0;
//       cmd_vel_pub.publish(cmd_vel);
//     }

//     if (backUpCounter == 20)
//     {
//       // Turn the robot around...
//       geometry_msgs::Twist cmd_vel;
//       cmd_vel.linear.x = 0.0;
//       cmd_vel.angular.z = 0.5;
//       cmd_vel_pub.publish(cmd_vel);
//     }

//     if (backUpCounter == 100)
//     {
//       // Stop the robot...
//       geometry_msgs::Twist cmd_vel;
//       cmd_vel.linear.x = 0.0;
//       cmd_vel.angular.z = 0.0;
//       cmd_vel_pub.publish(cmd_vel);
//       // ROS_INFO("Done backing up, now on with my life!");
//     }

//     backUpCounter++;
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// }

void PatrolAgent::do_interference_behavior()
{
  ROS_INFO("Interference detected! Executing interference behavior...\n");
  send_interference(); // send interference to monitor for counting

  // backup();

#if 1
      // Stop the robot..
  cancelGoal();
  ROS_INFO("Robot stopped");
  ros::Duration delay(1); // seconds
  delay.sleep();
  ResendGoal = true;
#else
  // get own "odom" positions...
  ros::spinOnce();

  //---------------
  // backup();
  //---------------

  // Waiting until conflict is solved...
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

// ROBOT-ROBOT COMMUNICATION

// void PatrolAgent::send_positions()
// {
//   // Publish Position to common node:
//   nav_msgs::Odometry msg;

//   int idx = ID_ROBOT;

//   if (ID_ROBOT <= -1)
//   {
//     msg.header.frame_id = "map"; // identificador do robot q publicou
//     idx = 0;
//   }
//   else
//   {
//     char string[20];
//     sprintf(string, "robot_%d/map", ID_ROBOT);
//     msg.header.frame_id = string;
//   }

//   msg.pose.pose.position.x = xPos[idx]; // send odometry.x
//   msg.pose.pose.position.y = yPos[idx]; // send odometry.y

//   positions_pub.publish(msg);
//   ros::spinOnce();
// }

void PatrolAgent::send_positions()
{
  //Publish Position to common node:
  //nav_msgs::Odometry msg;
  /*
    int idx = ID_ROBOT;

     if (ID_ROBOT <= -1){
        msg.header.frame_id = "map";    //identificador do robot q publicou
        idx = 0;
    }else{
        char string[20];
        sprintf(string,"robot_%d/map",ID_ROBOT);
        msg.header.frame_id = string;
    }

    msg.pose.pose.position.x = xPos[idx]; //send odometry.x
    msg.pose.pose.position.y = yPos[idx]; //send odometry.y

    positions_pub.publish(msg);
    */

  lastXpose = xPos[ID_ROBOT] * 100;
  lastYpose = yPos[ID_ROBOT] * 100;
  std_msgs::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(ID_ROBOT);
  msg.data.push_back(POSITION_MSG_TYPE);
  msg.data.push_back(lastXpose);
  msg.data.push_back(lastYpose);
  //msg.data.push_back(next_vertex);
  //msg.data.push_back(0); //David Portugal: is this necessary?
  //printf(" POSITION TO SEND x:%f y:%f \n",lastXpose,lastYpose);
  do_send_message(msg);

  //results_pub.publish(msg);
  //ros::spinOnce();

  ros::spinOnce();
}

void PatrolAgent::receive_positions()
{
}

void PatrolAgent::positionsCB(const nav_msgs::Odometry::ConstPtr &msg)
{ // construir tabelas de posições

  //     printf("Construir tabela de posicoes (receber posicoes), ID_ROBOT = %d\n",ID_ROBOT);

  char id[20]; // identificador do robot q enviou a msg d posição...
  strcpy(id, msg->header.frame_id.c_str());
  // int stamp = msg->header.seq;
  //     printf("robot q mandou msg = %s\n", id);

  // Build Positions Table

  if (ID_ROBOT > -1)
  {
    // verify id "XX" of robot: (string: "robot_XX/map")

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

    if (idx >= TEAMSIZE && TEAMSIZE <= NUM_MAX_ROBOTS)
    {
      // update teamsize:
      TEAMSIZE = idx + 1;
    }

    if (ID_ROBOT != idx)
    { // Ignore own positions
      xPos[idx] = msg->pose.pose.position.x;
      yPos[idx] = msg->pose.pose.position.y;
    }
    //      printf ("Position Table:\n frame.id = %s\n id_robot = %d\n xPos[%d] = %f\n yPos[%d] = %f\n\n", id, idx, idx,
    //      xPos[idx], idx, yPos[idx] );
  }

  receive_positions();
}

// simulates blocking send operation with delay in communication
void PatrolAgent::do_send_message(std_msgs::Int16MultiArray &msg)
{
  if (communication_delay > 0.001)
  {
    // double current_time = ros::Time::now().toSec();
    // if (current_time-last_communication_delay_time>1.0) {
    // ROS_INFO("Communication delay %.1f",communication_delay);
    ros::Duration delay(communication_delay); // seconds
    delay.sleep();
    // last_communication_delay_time = current_time;
    //}
  }
  results_pub.publish(msg);
  ros::spinOnce();
  // std::stringstream ss;

  // for (std::vector<signed short>::iterator it = msg.data.begin(); it != msg.data.end(); ++it)
  // {
  //   ss << *it << " ";
  // }
  // std::string s = ss.str();

  // //results_pub.publish(msg);
  // #if DBG
  // tcp_interface::RCOMMessage m;
  // m.header.stamp = ros::Time::now();
  // m.robotreceiver = "all";
  // m.robotsender = robotname;
  // m.value = s;
  // rcom_pub.publish(m);
  // #endif
  // ros::spinOnce();
}

void PatrolAgent::send_interference()
{
  // interference: [ID,msg_type]

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

  do_send_message(msg);

  // results_pub.publish(msg);
  // ros::spinOnce();
}

void PatrolAgent::send_resendgoal()
{
  // interference: [ID,msg_type]

  int value = ID_ROBOT;
  if (value == -1)
  {
    value = 0;
  }
  printf("Resend goal: Robot %d\n", value);

  std_msgs::Int16MultiArray msg;
  msg.data.clear();
  msg.data.push_back(value);
  msg.data.push_back(RESENDGOAL_MSG_TYPE);

  // results_pub.publish(msg);
  ros::spinOnce();
}

void PatrolAgent::resultsCB(const std_msgs::Int16MultiArray::ConstPtr &msg)
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

  // printf(" MESSAGE FROM %d TYPE %d ...\n",id_sender, msg_type);

  // messages coming from the monitor
  if (id_sender == -1 && msg_type == INITIALIZE_MSG_TYPE)
  {
    if (initialize == true && vresults[2] == 100)
    { //"-1,msg_type,100,seq_flag" (BEGINNING)
      ROS_INFO("Let's Patrol!\n");
      double r = 1.0 * ((rand() % 1000) / 1000.0);

      // TODO if sequential start
      // r = DELTA_TIME_SEQUENTIAL_START * ID_ROBOT;

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

#if DBG

// void PatrolAgent::resultsCB(const tcp_interface::RCOMMessage::ConstPtr &msg)
// {

//   //std::vector<signed short>::const_iterator it = msg->data.begin();

//   vresults.clear();

//   //for (size_t k=0; k<msg->data.size(); k++) {
//   //    vresults.push_back(*it); it++;
//   //}

//   signed short buf;

//   string message;
//   message = msg->value;
//   stringstream ss(message); // Insert the string into a stream

//   //printf(" MESSAGE RECEIVED %s \n",message.c_str());
//   while (ss >> buf)
//   {
//     //tokens.push_back(buf);
//     vresults.push_back(buf);
//   }

//   int id_sender = vresults[0];
//   int msg_type = vresults[1];

//   if (msg->robotreceiver != robotname) //&& msg->robotreceiver!="all")
//     return;

//   //printf("--> MESSAGE FROM %d To %s/%s TYPE %d ...\n",id_sender,
//   //   msg->robotreceiver.c_str(),robotname.str().c_str(), msg_type);

//   if (id_sender != ID_ROBOT && msg_type == POSITION_MSG_TYPE)
//   {
//     if (id_sender >= TEAMSIZE && TEAMSIZE < NUM_MAX_ROBOTS)
//     {
//       //update teamsize:
//       TEAMSIZE = id_sender + 1;
//     }
//     //printf(" POSITION RECEIVED in MM FROM %d x:%d y:%d \n",id_sender,vresults[2],vresults[3]);
//     xPos[id_sender] = float(vresults[2]) / 100;
//     yPos[id_sender] = float(vresults[3]) / 100;
//     //printf(" POSITION RECEIVED FROM %d x:%f y:%f \n",id_sender,xPos[id_sender],yPos[id_sender]);
//     receive_positions();
//   }
//   else
//   {

//     // messages coming from the monitor
//     if (id_sender == -1 && msg_type == INITIALIZE_MSG_TYPE)
//     {
//       if (initialize == true && vresults[2] == 100)
//       { //"-1,msg_type,100" (BEGINNING)
//         ROS_INFO("Let's Patrol!\n");
//         double r = 1.0 * ((rand() % 1000) / 1000.0);
//         ros::Duration wait(r); // seconds
//         wait.sleep();
//         initialize = false;
//       }

//       if (initialize == false && vresults[2] == 999)
//       { //"-1,msg_type,999" (END)
//         ROS_INFO("The simulation is over. Let's leave");
//         end_simulation = true;
//       }
//     }

//     if (!initialize)
//     {
// #if 0
//             // communication delay
//             if ((communication_delay>0.001) && (id_sender!=ID_ROBOT)) {
//                 double current_time = ros::Time::now().toSec();
//                 if (current_time-last_communication_delay_time>1.0) {
//                     ROS_INFO("Communication delay %.1f",communication_delay);
//                     ros::Duration delay(communication_delay); // seconds
//                     delay.sleep();
//                     last_communication_delay_time = current_time;
//                 }
//             }
//             bool lost_message = false;
//             if ((lost_message_rate>0.0001)&& (id_sender!=ID_ROBOT)) {
//                 double r = (rand() % 1000)/1000.0;
//                 lost_message = r < lost_message_rate;
//             }
//             if (lost_message) {
//                 ROS_INFO("Lost message");
//             }
//             else
// #endif
//       receive_results();
//     }
//   }
//   ros::spinOnce();
// }


void PatrolAgent::receive_mission_Callback(const task_planner::TaskConstPtr &msg)
{
  if (msg->ID_ROBOT == ID_ROBOT)
  {
    if (!msg->go_home)
    {
      c_print("@ Task ricevuto! n: ", msg->order, " ID_ROBOT = ", ID_ROBOT, green);
      Task task;
      task.take = msg->take;
      task.item = msg->item;
      task.order = msg->order;
      task.demand = msg->demand;
      task.dst = msg->dst;
      task.path_distance = msg->path_distance;
      // Route step;
      for (auto i = 0; i < msg->route.size(); i++)
      {
        // step.id_vertex = msg->route[i];
        // step.status = msg->condition[i];
        task.trail.push_back(msg->route[i]);
      }
      c_print("# insert task on mission!", red, Pr);
      mission.push_back(task);
      //  end_simulation = false;
    }
    else
    {
      uint elem_s_path;
      int *shortest_path = new int[dimension];
      uint home = msg->dst;
      dijkstra(current_vertex, home, shortest_path, elem_s_path, vertex_web, dimension);
      Task t;
      t.take = msg->take;
      t.dst = msg->dst; //aggiunta destinazione casa al task
      // Route step;
      for (auto i = 2; i < elem_s_path; i++)
      {
        // printf("path[%u] = %d\n", i, shortest_path[i]);
        // step.status = false;
        t.trail.push_back(shortest_path[i]);
      }
      int pd = ccor(t.trail);
      t.path_distance = pd;
      c_print("# insert task to go home!", magenta);
      mission.push_back(t);
    }
  }
}

#endif

void PatrolAgent::init_Callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  // dopo aver mandato il messaggio che ho bisogno di un task aspetto finche il TP non mi risponde.
  // appena mi risponde allora posso partire
}

void PatrolAgent::broadcast_msg_Callback(const std_msgs::Int16MultiArray::ConstPtr &msg)
{
  // molto importante <signed short>
  std::vector<signed short>::const_iterator it = msg->data.begin();

  int id_sender = *it;
  it++;
  int value = ID_ROBOT;
  c_print("message id robot: ", ID_ROBOT, green, Pr);
  if (value == -1)
  {
    value = 0;
  }
  if (id_sender == value)
    return;

  int msg_type = *it;
  it++;
  switch (msg_type)
  {
  case (INIT_MSG):
  {
    if (value == 0)
    {
      OK = true;
    }
  }
  break;
  case (START):
  {
    c_print("PArte quello dopo", red, Pr);
    if (value == *it)
    {
      OK = true;
    }
  }
  break;
  }
  ros::spinOnce();
}

} // namespace patrolagent
