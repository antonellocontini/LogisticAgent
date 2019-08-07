#pragma once
#include "patrolling_sim/Token.h"

namespace tpagent
{

void TPAgent::init(int argc, char **argv)
{
    PatrolAgent::init(argc, argv);
    ros::NodeHandle nh;

    token_pub = nh.advertise<patrolling_sim::Token>("token_msg", 1);

    token_sub = nh.subscribe<patrolling_sim::Token>("token_msg", 20,
                                                    boost::bind(&TPAgent::token_callback, this, _1));

    init_tw_map();

    reached_pickup = false;
}

void TPAgent::run()
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


void TPAgent::onGoalComplete()
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


int TPAgent::compute_next_vertex()
{
    c_print("[DEBUG]\tCalled compute_next_vertex()", yellow);
    int vertex;

    while(mission.empty())
    {
        c_print("[WARN]\tMission not received! Waiting...", yellow);
        usleep(200000);
    }

    assert(!mission.empty());
    if (current_vertex == 6)
    {
        c_print("[DEBUG]\tReached pickup point", yellow);
        reached_pickup = true;
    }

    if (current_vertex == mission[id_task].dst && reached_pickup)
    // if (current_vertex == mission[id_task].trail.back())
    {
        if (mission[id_task].take)
        {
            c_print("RequestTask", green);
            request_Task();
            reached_pickup = false;
            sleep(10);
        }

        send_task_reached();
    }

    int path[dimension];
    uint path_length;
    if (!reached_pickup)
    {
        c_print("[DEBUG]\tCalling tp_dijkstra, first leg", yellow);
        tp_dijkstra(current_vertex, 6, path, path_length);
        // dijkstra(current_vertex, mission[id_task].dst, path, path_length, vertex_web, dimension);
    }
    else
    {
        c_print("[DEBUG]\tCalling tp_dijkstra, last leg", yellow);
        tp_dijkstra(current_vertex, mission[id_task].dst, path, path_length);
        // dijkstra(current_vertex, mission[id_task].trail.back(), path, path_length, vertex_web, dimension);
    }
    vertex = path[1];

    c_print("[DEBUG]\tpath_length: ", path_length, yellow);
    for(int i=0; i<path_length; i++)
    {
        c_print("\t\t",path[i], yellow);
    }
    c_print("current vertex: ", current_vertex, "\tnext vertex: ", vertex, "\tdestination: ", mission[id_task].dst, magenta);
    return vertex;
} // compute_next_vertex()


void TPAgent::tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path)
{
    const int PENALTY=500;

    vertex web_copy[dimension];
    for( int i=0; i<dimension; i++ )
    {
        // copia della struttura
        // web_copy[i] = vertex_web[i];
        web_copy[i].id = vertex_web[i].id;
        web_copy[i].num_neigh = vertex_web[i].num_neigh;
        web_copy[i].x = vertex_web[i].x;
        web_copy[i].y = vertex_web[i].y;
        memcpy( web_copy[i].id_neigh, vertex_web[i].id_neigh, sizeof(uint)*8 );
        memcpy( web_copy[i].cost, vertex_web[i].cost, sizeof(uint)*8 );
        memcpy( web_copy[i].cost_m, vertex_web[i].cost_m, sizeof(float)*8 );
        memcpy( web_copy[i].visited, vertex_web[i].visited, sizeof(bool)*8 );
        for( int j=0; j<web_copy[i].num_neigh; j++)
        {
            memcpy( web_copy[i].dir[j], vertex_web[i].dir[j], sizeof(char)*3 );
        }

        // incremento il peso degli archi occupati
        for( int j=0; j<web_copy[i].num_neigh; j++ )
        {
            uint from = web_copy[i].id;
            uint to = web_copy[i].id_neigh[j];
            web_copy[i].cost[j] += PENALTY*token_weight_map[from][to];
        }
    }

    // calcolo il percorso con il nuovo grafo
    dijkstra( source, destination, shortest_path, elem_s_path, web_copy, dimension );
}


void TPAgent::token_callback(const patrolling_sim::TokenConstPtr &msg)
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
    else if (ID_ROBOT == (msg->ID_ROBOT + 1) % (CAPACITY+1))
    {
        std::ostringstream oss;
        oss << "[DEBUG]\tToken ricevuto! ID messaggio: " << msg->ID_ROBOT
            << "\tID robot: " << ID_ROBOT << "\tTEAMSIZE: " << CAPACITY+1;
        std::string s = oss.str();
        // c_print(s.c_str(), yellow);

        // riempo campi messaggio
        patrolling_sim::Token t;
        t.ID_ROBOT = ID_ROBOT;
        t.INIT_DONE = true;
        t.TEAMSIZE = CAPACITY+1;
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


void TPAgent::init_tw_map()
{
    token_weight_map.clear();
    for (int i = 0; i < dimension; i++)
    {
        std::vector<uint> v;
        for (int j = 0; j < dimension; j++)
        {
            v.push_back(0);
        }
        token_weight_map.push_back(v);
    }
}

} //namespace tpagent