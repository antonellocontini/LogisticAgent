#include "SPartAgent.hpp"

using namespace spartagent;

logistic_sim::Mission SPartAgent::coalition_formation(logistic_sim::Token &token)
{
    auto mission = token.MISSION.front();
    token.MISSION.erase(find(token.MISSION.begin(), token.MISSION.end(), mission));
    mission.PICKUP = true;
    return mission;
}

void SPartAgent::run()
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

        if (t_interference)
        {
            //do_interference_behavior();
            // do_interference_behavior();
            // invece di eseguire il comportamento di interferenza
            // provo a dire al robot di andare al vertice da dove è arrivato
            uint temp = next_vertex;
            next_vertex = current_vertex;
            current_vertex = temp;
            // se non setto interference a false questo ramo viene eseguito un paio
            // di volte poichè il token deve completare il giro prima che la variabile
            // interference venga calcolata
            t_interference = 0;
            sendGoal(next_vertex);
            c_print("ID_ROBOT: ", ID_ROBOT, "\tInterferenza rilevata, vado in ", next_vertex, red, P);
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


int main(int argc, char *argv[])
{
    spartagent::SPartAgent SPA;
    SPA.init(argc, argv);
    c_print("@ SPART", green);
    sleep(3);
    SPA.run();
    return 0;
}