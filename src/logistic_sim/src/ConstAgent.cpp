#include "ConstAgent.hpp"

using namespace constagent;

std::pair<int,int> ConstAgent::check_interference_token(logistic_sim::Token &token)
{
    int i;
    double dist_quad;
    // TEST DISTANZA
    // for (i=0; i<TEAM_SIZE; i++)
    // {
    //     dist_quad = (token.X_POS[i] - token.X_POS[ID_ROBOT]) * (token.X_POS[i] - token.X_POS[ID_ROBOT]) + (token.Y_POS[i] - token.Y_POS[ID_ROBOT]) * (token.Y_POS[i] - token.Y_POS[ID_ROBOT]);
    //     if (i != ID_ROBOT && dist_quad <= INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE)
    //     {
    //         c_print("LAST_INTERFERENCE= ", last_interference, red, P);
    //         c_print("SEQ= ",token.HEADER.seq,"\tSONO VICINO A ", i,":\tDIST= ",dist_quad, red, P);
    //     }
    // }

    if (ros::Time::now().toSec() - last_interference < 10) // seconds
    {
        return std::pair<int,int>(t_interference, id_interference); // false if within 10 seconds from the last one
    }

    /* Poderei usar TEAMSIZE para afinar */
    // ID_ROBOT
    for (i = 0; i < TEAM_SIZE; i++)
    { //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)

        // c_print("MY_POSITION ", token.X_POS[ID_ROBOT], " ", token.Y_POS[ID_ROBOT], red, P);
        // c_print("OTHER_POSITION ", token.X_POS[i], " ", token.Y_POS[i], red, P);
        dist_quad = (token.X_POS[i] - token.X_POS[ID_ROBOT]) * (token.X_POS[i] - token.X_POS[ID_ROBOT]) + (token.Y_POS[i] - token.Y_POS[ID_ROBOT]) * (token.Y_POS[i] - token.Y_POS[ID_ROBOT]);
        // c_print("SEQ:\t",token.HEADER.seq,"\tDISTANCE WITH ",i,":\t",dist_quad,red,P);

        if (i != ID_ROBOT && dist_quad <= INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE)
        { //robots are ... meter or less apart

            //          ROS_INFO("Feedback: Robots are close. INTERFERENCE! Dist_Quad = %f", dist_quad);
            ros::Duration my_delta_time_mission = token.HEADER.stamp.now() - token.MISSION_START_TIME[ID_ROBOT];
            ros::Duration other_delta_time_mission = token.HEADER.stamp.now() - token.MISSION_START_TIME[i];

            float my_mission_distance = token.MISSION_CURRENT_DISTANCE[ID_ROBOT];
            float other_mission_distance = token.MISSION_CURRENT_DISTANCE[i];

            float my_metric = my_mission_distance / my_delta_time_mission.sec;
            float other_metric = other_mission_distance / other_delta_time_mission.sec;

            double x_dst = vertex_web[current_mission.DSTS[0]].x;
            double y_dst = vertex_web[current_mission.DSTS[0]].y;
            double other_x_dst = vertex_web[token.CURR_DST[i]].x;
            double other_y_dst = vertex_web[token.CURR_DST[i]].y;
            double other_distance = (token.X_POS[i] - other_x_dst) * (token.X_POS[i] - other_x_dst) + (token.Y_POS[i] - other_y_dst) * (token.Y_POS[i] - other_y_dst);
            double my_distance = (token.X_POS[ID_ROBOT] - x_dst) * (token.X_POS[ID_ROBOT] - x_dst) + (token.Y_POS[ID_ROBOT] - y_dst) * (token.Y_POS[ID_ROBOT] - y_dst);
            // c_print("my_distance ", my_distance, "\tother_distance ", other_distance, yellow);

            // TEST
            // c_print("[DEBUG]\tTEST", red, P);
            // token.INTERFERENCE_COUNTER[ID_ROBOT]++;
            // token.INTERFERENCE_STATUS[ID_ROBOT] = 1;
            // return std::pair<int,int>(1, i);

            c_print("Metric boolean: ", my_distance > other_distance, red, P);
            if (my_distance > other_distance)
            {
                last_interference = ros::Time::now().toSec();
                token.INTERFERENCE_COUNTER[ID_ROBOT]++;
                if (current_vertex == token.NEXT_VERTEX[i])
                {
                    token.INTERFERENCE_STATUS[ID_ROBOT] = 1;
                    return std::pair<int,int>(1, i);
                }
                else
                {
                    token.INTERFERENCE_STATUS[ID_ROBOT] = 2;
                    return std::pair<int,int>(2, i);
                }
            }
            else
            {
                c_print("[DEBUG]\tDovrebbe andare ROBOT_ID: ", i, " in interferenza", red, P);
            }
        }
    }
    token.INTERFERENCE_STATUS[ID_ROBOT] = 0;
    return std::pair<int, int>(0, 0);
}

void ConstAgent::run()
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

    // TEST: attesa di qualche secondo dalla partenza del precedente
    init_start_time = ros::Time::now();
    init_wait_time = 13 * ID_ROBOT;
    c_print("[DEBUG]\tAttendo ", init_wait_time, " secondi...", yellow);
    // sleep(wait_time);
    // c_print("[DEBUG]\tParto");

    // Asynch spinner (non-blocking)
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();
    // ros::waitForShutdown();

    /* Run Algorithm */

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

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
        // if (goal_complete)
        // {
        //     c_print("before OnGoal()", magenta);
        //     onGoalComplete();
        //     resend_goal_count = 0;
        // }
        uint temp = 0;
        bool clear = true;
        switch (t_interference)
        {
        case 0:
            // no interference
            break;
        case 1:
            temp = next_vertex;
            next_vertex = current_vertex;
            current_vertex = temp;
            t_interference = 0;
            sendGoal(next_vertex);
            c_print("ID_ROBOT: ", ID_ROBOT, "\tInterferenza rilevata, vado in ", next_vertex, red, P);
            break;
        case 2:
            // caso wait 5 sec
            cancelGoal();
            c_print("ID_ROBOT: ", ID_ROBOT, "\tInterferenza rilevata, wait stay in ", next_vertex, green, P);

            do
            {
                clear = true;
                if (id_interference != ID_ROBOT && !check_neighbour_dist(id_interference, INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE))
                {
                    clear = false;
                }
                usleep(10000);
            } while (!clear);
            
            sendGoal(next_vertex);
            t_interference = 0;
            break;
        default:
            c_print("default");
            break;
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

        loop_rate.sleep();

    } // while ros.ok
} // run()

// -----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    constagent::ConstAgent CA;
    CA.init(argc, argv);
    c_print("@ CONST", green);
    sleep(3);
    CA.run();
    return 0;
}