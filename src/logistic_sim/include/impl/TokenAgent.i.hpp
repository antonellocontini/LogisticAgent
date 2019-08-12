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
    home_vertex = current_vertex;
    c_print("current_vertex: ", current_vertex, red);
    c_print("home_vertex: ", home_vertex, red);
    c_print("Team size: ", num_robots, red);
    sleep(5);

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    //il primo robot fa partire l'anello
    if (ID_ROBOT == 0)
    {
        c_print("Avvio token ring", magenta);
        c_print("RequestTask", green);
        request_Task();
        sleep(5);

        patrolling_sim::Token t;
        t.ID_ROBOT = 0;
        t.TEAMSIZE = TEAMSIZE;
        t.INIT_DONE = true;
        t.ID_ROBOT_VERTEX.push_back(0);
        t.SRC_VERTEX.push_back(current_vertex);
        t.DST_VERTEX.push_back(next_vertex);
        // t.FIRST_ROUND = true;
        token_pub.publish(t);
    }
    else
    {
        c_print("RequestTask", green);
        request_Task();
        sleep(5);
    }

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
    // c_print("[DEBUG]\tCalled token_callback()", yellow);
    // calcolo dimensione team
    if (!msg->INIT_DONE)
    {
        if (msg->TEAMSIZE > TEAMSIZE)
        {
            TEAMSIZE = msg->TEAMSIZE;
        }
    }
    else if (ID_ROBOT == (msg->ID_ROBOT + 1) % num_robots)
    {
        std::ostringstream oss;
        oss << "[DEBUG]\tToken ricevuto! ID messaggio: " << msg->ID_ROBOT
            << "\tID robot: " << ID_ROBOT << "\tTEAMSIZE: " << num_robots;
        std::string s = oss.str();
        // c_print(s.c_str(), yellow);

        // riempo campi messaggio
        patrolling_sim::Token t;
        t.ID_ROBOT = ID_ROBOT;
        t.INIT_DONE = true;
        t.TEAMSIZE = num_robots;
        t.ID_ROBOT_VERTEX = msg->ID_ROBOT_VERTEX;
        t.SRC_VERTEX = msg->SRC_VERTEX;
        t.DST_VERTEX = msg->DST_VERTEX;
        // c_print("[DEBUG]\tCampi riempiti", yellow);

        // aggiorno le informazioni sull'arco che sto occupando
        bool found = false;
        for (auto i = 0; i < t.ID_ROBOT_VERTEX.size() && !found; i++)
        {
            if (t.ID_ROBOT_VERTEX[i] == ID_ROBOT)
            {
                t.SRC_VERTEX[i] = current_vertex;
                t.DST_VERTEX[i] = next_vertex;
                found = true;
            }
        }

        // il mio id non è ancora stato inserito nel token
        if (!found)
        {
            t.ID_ROBOT_VERTEX.push_back(ID_ROBOT);
            t.SRC_VERTEX.push_back(current_vertex);
            t.DST_VERTEX.push_back(next_vertex);
        }
        // c_print("[DEBUG]\tToken aggiornato", yellow);

        // aggiorno la mappa degli archi occupati
        init_tw_map();
        // c_print("[DEBUG]\tinit_tw_map() terminato\n\tAggiornamento mappa archi...", yellow);
        for (auto i=0; i<t.ID_ROBOT_VERTEX.size(); i++)
        {
            int dst = (int) t.DST_VERTEX[i];
            int src = (int) t.SRC_VERTEX[i];
            if (dst >= 0 && dst < dimension &&
                src >= 0 && src < dimension &&
                t.ID_ROBOT_VERTEX[i] != ID_ROBOT)
            {
                // c_print("\t\ti:",i,"\tsrc: ",t.SRC_VERTEX[i],"\tdst: ",t.DST_VERTEX[i], yellow);

                //svaforisco la mia direzione
                // token_weight_map[t.SRC_VERTEX[i]][t.DST_VERTEX[i]]++;
                //sfavorisco la direzione inversa
                token_weight_map[t.DST_VERTEX[i]][t.SRC_VERTEX[i]]+=3;
            }
        }
        // c_print("[DEBUG]\tMappa archi aggiornata", yellow);

        ros::Duration d(0.1);
        d.sleep();
        token_pub.publish(t);
        // c_print("Token pubblicato!", green);
    }
    
    // c_print("[DEBUG]\ttoken_callback() terminated", yellow);
} // token_callback()



} //namespace tpagent