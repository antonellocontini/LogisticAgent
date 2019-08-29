#include "DistrAgent.hpp"

using namespace distragent;

void DistrAgent::init(int argc, char **argv)
{
    Agent::init(argc, argv);

    // inizializzazione della tmp_CAPCACITY
    tmp_CAPACITY = CAPACITY;

    ros::NodeHandle nh;

    token_pub = nh.advertise<logistic_sim::Token>("token", 1);

    token_sub = nh.subscribe<logistic_sim::Token>("token", 20, boost::bind(&DistrAgent::token_callback, this, _1));

    init_tw_map();
}

void DistrAgent::run()
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

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    // TEST: attesa di qualche secondo dalla partenza del precedente
    int wait_time = 10 * ID_ROBOT;
    c_print("[DEBUG]\tAttendo ", wait_time, " secondi...", yellow);
    sleep(wait_time);
    c_print("[DEBUG]\tParto");

    while (ros::ok())
    {

        //  cosa ho globale??? mi serve tempo velocita e posizione
        // float x, y, t;
        // getRobotPose(ID_ROBOT, x, y, t);
        // std::cout << "x: " << xPos[ID_ROBOT] << " y: " << yPos[ID_ROBOT] << " th: " << thetaPos[ID_ROBOT] << "\n"
        //           << "lastPos: x= " << lastXpose << " ,y= " << lastYpose << "\n";

        // lastXPose
        // lastYpose
        // dimension
        // initial_vertex
        // current_vertex
        // next_vertex
        // last_interference
        // std::cout <<
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

void DistrAgent::onGoalComplete()
{
    if (next_vertex > -1)
    {
        current_vertex = next_vertex;
    }

    c_print("[DEBUG]\tgo_src(): ", go_src(), "\tgo_dst(): ", go_dst(), yellow);
    c_print("\t\tcurrent_vertex: ", current_vertex, yellow);
    for (auto it = current_mission.DSTS.begin(); it != current_mission.DSTS.end(); it++)
    {
        c_print("\t\t\tDSTS ", (*it), magenta, P);
    }
    // aggiorniamo condizioni destinazione
    if (go_home && current_vertex == initial_vertex)
    {
        need_task = true;
    }
    else if (go_src() && current_vertex == 6)
    {
        current_mission.PICKUP = false;
        tmp_CAPACITY -= current_mission.TOT_DEMAND;
    }
    else if (go_dst() && current_vertex == current_mission.DSTS[0])
    {
        c_print("Capacity before unloading: ", tmp_CAPACITY, red);
        uint d = current_mission.DEMANDS[0];
        current_mission.DSTS.erase(current_mission.DSTS.begin());
        current_mission.DEMANDS.erase(current_mission.DEMANDS.begin());
        current_mission.TOT_DEMAND -= d;
        tmp_CAPACITY += d;
        if (current_mission.DSTS.empty())
        {
            need_task = true;
        }

        c_print("d: ", d, red);
        c_print("Current capacity: ", tmp_CAPACITY, red);
    }

    // if (tmp_CAPACITY > 0)
    // {
    //     need_task = true;
    // }
    // else
    // {
    //     need_task = false;
    // }

    c_print("before compute_next_vertex()", yellow);
    next_vertex = compute_next_vertex();

    c_print("   @ compute_next_vertex: ", next_vertex, green);

    send_goal_reached(); // Send TARGET to monitor

    send_results(); // Algorithm specific function

    // Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    // sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    sendGoal(next_vertex); // send to move_base

    goal_complete = false;
}

int DistrAgent::compute_next_vertex()
{
    int vertex;

    // alloco un vettore per il percorso
    int path[dimension];
    uint path_length;

    if (go_home)
    {
        c_print("[DEBUG]\tgoing_home...\tinitial_vertex: ", initial_vertex, yellow);
        tp_dijkstra(current_vertex, initial_vertex, path, path_length);
    }
    else if (go_src())
    {
        tp_dijkstra(current_vertex, 6, path, path_length);
    }
    else if (go_dst())
    {
        tp_dijkstra(current_vertex, current_mission.DSTS[0], path, path_length);
    }

    c_print("[DEBUG]\tpath_length: ", path_length, "\tpath:", yellow);
    for (int i = 0; i < path_length; i++)
    {
        c_print("\t\t", path[i], yellow);
    }

    if (path_length > 1)
    {
        vertex = path[1]; //il primo vertice è quello di partenza, ritorno il secondo
    }
    else
    {
        vertex = current_vertex;
    }

    return vertex;
} // compute_next_vertex()

bool DistrAgent::go_src()
{
    return current_mission.PICKUP;
}

bool DistrAgent::go_dst()
{
    return !go_src() && !current_mission.DSTS.empty();
}

// -----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    distragent::DistrAgent TPA;
    TPA.init(argc, argv);
    c_print("@ Inizializzazione finita!", green);
    sleep(3);
    TPA.run();
    return 0;
}