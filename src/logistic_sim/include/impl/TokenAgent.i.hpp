#pragma once
#include "patrolling_sim/Token.h"

namespace tokenagent
{

void TokenAgent::init(int argc, char **argv)
{
    Agent::init(argc, argv);
    ros::NodeHandle nh;

    token_pub = nh.advertise<patrolling_sim::Token>("token_msg", 1);

    token_sub = nh.subscribe<patrolling_sim::Token>("token_msg", 20,
                                                    boost::bind(&TokenAgent::token_callback, this, _1));
}

void TokenAgent::run()
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
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();
    // ros::waitForShutdown();

    /* Run Algorithm */

    //  init_agent3();

    // c_print("RequestTask", green);
    // request_Task();

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    while (ros::ok())
    {
        if (goal_complete)
        {
            c_print("before OnGoal()", magenta);
            onGoalComplete();
            resend_goal_count = 0;
        }
        else
        {
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

                ResendGoal = false;
            }

            processEvents();

            if (end_simulation)
            {
                return;
            }
        }

        loop_rate.sleep();

    } // while ros.ok
} // run()


void TokenAgent::onGoalComplete()
{
    if (next_vertex > -1)
    {
        current_vertex = next_vertex;
    }

    c_print("compute_next_vertex", yellow);
    next_vertex = compute_next_vertex();

    c_print("   @ compute_next_vertex: ", next_vertex, green);

    c_print("[DEBUG]\tid_task: ", id_task, "\tdst: ", mission[id_task].dst, "\tnext_vertex: ", next_vertex, yellow);

    send_goal_reached(); // Send TARGET to monitor

    send_results(); // Algorithm specific function

    // Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    // sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    sendGoal(next_vertex); // send to move_base

    goal_complete = false;
}


int TokenAgent::compute_next_vertex()
{
    int vertex = 0;
    return vertex;
} // compute_next_vertex()


void TokenAgent::token_callback(const patrolling_sim::TokenConstPtr &msg)
{
    
    // c_print("[DEBUG]\ttoken_callback() terminated", yellow);
} // token_callback()



} //namespace tpagent