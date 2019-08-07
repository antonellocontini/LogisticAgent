#pragma once

namespace coa
{
using namespace std;
int COA::compute_next_vertex()
{
  int vertex;

  if (mission[id_task].trail.size() - 1 == id_vertex)
  {
    vertex = mission[id_task].trail[id_vertex];
    if (mission[id_task].take)
    {
      // request_Mission();
      // sleep(10);
    }
    else
    {
      // pubblico un messaggio al view_result e gli dico sono a casa. quando tutti hanno mandato il messaggio scrivo i
      // risultati e end_simulation.
      int value = ID_ROBOT;
      if (value == -1)
      {
        value = 0;
      }

      // [ID,msg_type,vertex,intention,0]
      c_print("ATHOMEEEEEEEEEEEEEEEEEEEEEEEEEEEEE", red, Pr);
      std_msgs::Int16MultiArray msg;
      msg.data.clear();
      msg.data.push_back(value);
      msg.data.push_back(AT_HOME_MSG_TYPE);
      // results_pub.publish(msg);
      ros::spinOnce();

      at_home = true;
    }
    //  ^ Importatnte!
    // mission.clear();
    c_print("id_v: ", id_vertex, " vertex: ", vertex, magenta);
    send_task_reached();
    id_vertex = 0;
  }
  else
  {
    vertex = mission[id_task].trail[id_vertex];
    c_print("id_v: ", id_vertex, " vertex: ", vertex, yellow);
    id_vertex++;
  }

  return vertex;
}

void COA::onGoalComplete()
{
  if (next_vertex > -1)
  {
    // Update Idleness Table:
    // update_idleness();
    current_vertex = next_vertex;
  }

  // devolver proximo vertex tendo em conta apenas as idlenesses;

  if (!at_home)
    next_vertex = compute_next_vertex();
  else
  {
    c_print("sono a casa!", yellow, Pr);
    sendGoal(next_vertex);
    end_simulation = true;
  }

  // next_vertex = compute_next_vertex();

  c_print("   @ compute_next_vertex: ", next_vertex, green);

  // printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex,
  // vertex_web[next_vertex].x, vertex_web[next_vertex].y);

  /** SEND GOAL (REACHED) AND INTENTION **/

  send_goal_reached();  // Send TARGET to monitor

  send_results();  // Algorithm specific function

  // Send the goal to the robot (Global Map)
  ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
  // sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
  sendGoal(next_vertex);  // send to move_base

  goal_complete = false;
}

void COA::run()
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
    mb_string = "robot_" + id_string.str() + "/";
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
    // init_agent();

    if (goal_complete)
    {
      onGoalComplete();  // can be redefined
      resend_goal_count = 0;
    }
    else
    {  // goal not complete (active)
      if (interference)
      {
        do_interference_behavior();
      }

      if (ResendGoal)
      {
        ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex, vertex_web[next_vertex].x,
                 vertex_web[next_vertex].y);
        send_resendgoal();
        sendGoal(next_vertex);

        ResendGoal = false;  // para nao voltar a entrar (envia goal so uma vez)
      }

      processEvents();

      if (end_simulation)
      {
        return;
      }

    }  // if (goal_complete)
    // }// if (initialization)

    loop_rate.sleep();

  }  // while ros.ok
}

}  // cycle opt agent