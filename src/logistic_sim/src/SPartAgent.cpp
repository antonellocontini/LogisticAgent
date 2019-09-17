#include "SPartAgent.hpp"

using namespace spartagent;

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


void SPartAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
    // ricevo il token ricevo il task set computo la CF migliore la assegno e toglo i task che la compongono.

    // se non è per me termino
    if (msg->ID_RECEIVER != ID_ROBOT)
        return;

    logistic_sim::Token token;
    token = *msg;

    // logistic_sim::Mission m = coalition_formation(token);

    // cout << "missione finale dopo coalizione: "<< m.ID << m.TOT_DEMAND <<"\n";

    token.ID_SENDER = ID_ROBOT;
    // c_print("TEAMSIZE: ",TEAM_SIZE, yellow);
    if (msg->ID_RECEIVER == TEAM_SIZE - 1)
    {
        token.ID_RECEIVER = TASK_PLANNER_ID;
    }
    else
    {
        token.ID_RECEIVER = (ID_ROBOT + 1) % TEAM_SIZE;
    }

    if (msg->INIT)
    {
        token.CAPACITY.push_back(CAPACITY);
        // all avvio indico quale arco i robot vorrebbero attraversare
        // serve a forzare la partenza nella stessa direzione
        if (ID_ROBOT > 0)
        {
            init_next_vertex = token.CURR_VERTEX[ID_ROBOT - 1];
        }
        else
        {
            init_next_vertex = current_vertex;
        }
        token.CURR_VERTEX.push_back(current_vertex);
        token.NEXT_VERTEX.push_back(init_next_vertex);
        // solo INIT
        token.CURR_DST.push_back(dimension + 1);
        token.INIT_POS.insert(token.INIT_POS.begin(), initial_vertex);
        token.MISSION_START_TIME.push_back(ros::Time::now());
        token.MISSION_CURRENT_DISTANCE.push_back(0.0f);
        token.INTERFERENCE_COUNTER.push_back(0);
        token.MISSIONS_COMPLETED.push_back(0);
        token.TASKS_COMPLETED.push_back(0);
        token.TOTAL_DISTANCE.push_back(0.0f);
        token.INTERFERENCE_STATUS.push_back(0);
        token.X_POS.push_back(0.0);
        token.Y_POS.push_back(0.0);
        initialize = false;
    }
    else if (ros::Time::now().sec - init_start_time.sec >= init_wait_time)
    {

        // aggiorno posizione
        token.X_POS[ID_ROBOT] = xPos[ID_ROBOT];
        token.Y_POS[ID_ROBOT] = yPos[ID_ROBOT];
        for(int i=0; i<TEAM_SIZE; i++)
        {
            if (i != ID_ROBOT)
            {
                xPos[i] = token.X_POS[i];
                yPos[i] = token.Y_POS[i];
            }
        }

        if (need_task)
        {
            if (!token.MISSION.empty())
            {
                c_print("[DEBUG]\tsize before coalition: ", token.MISSION.size(), "\tcapacity: ", tmp_CAPACITY, yellow);

                current_mission = coalition_formation(token);
                c_print("ID: ", current_mission.ID, red, P);
                c_print("[DEBUG]\tsize after oalition: ", token.MISSION.size(), yellow);
                token.MISSION_START_TIME[ID_ROBOT] = token.HEADER.stamp.now();
                token.MISSION_CURRENT_DISTANCE[ID_ROBOT] = 0.0f;
            }
            else
            {
                go_home = true;
                initial_vertex = token.INIT_POS.back();
            }
            need_task = false;
            goal_complete = true;
        }

        if (go_home)
        {
            auto it = std::min_element(token.INIT_POS.begin(), token.INIT_POS.end());
            if (initial_vertex != *it)
            {
                initial_vertex = *it;
                goal_complete = true;
            }

            if (current_vertex == *it)
            {
                token.INIT_POS.erase(it);
                go_home = false;
                need_task = false;
            }
        }

        init_tw_map();
        // c_print("[DEBUG]\tinit_tw_map() terminato\n\tAggiornamento mappa archi...", yellow);
        for (auto i = 0; i < TEAM_SIZE; i++)
        {
            int dst = (int)token.NEXT_VERTEX[i];
            int src = (int)token.CURR_VERTEX[i];
            if (dst >= 0 && dst < dimension &&
                src >= 0 && src < dimension &&
                i != ID_ROBOT)
            {
                // la penalità dipende da quanto tempo sono sull'arco
                int sec_diff = ros::Time::now().sec - goal_start_time.sec;
                sec_diff = std::max(1, sec_diff);
                // c_print("[DEBUG]\ttw_map updated: [", src, ",", dst, "]", yellow);
                // svaforisco la mia direzione
                token_weight_map[src][dst] += sec_diff;
                // sfavorisco la direzione inversa
                token_weight_map[dst][src] += sec_diff * 4;
                // sfavorisco tutti gli archi che entrano nella mia destinazione
                // dovrebbe prevenire gli scontri agli incroci dove due robot
                // arrivano da nodi diversi
                for (int j = 0; j < dimension; j++)
                {
                    if (j != src)
                    {
                        token_weight_map[j][dst] += sec_diff * 2;
                    }
                }
            }
        }

        // metto nel token quale arco sto occupando
        token.CURR_VERTEX[ID_ROBOT] = current_vertex;
        token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
        token.CURR_DST[ID_ROBOT] = current_mission.DSTS[0];

        std::pair<int,int> interf_pair = check_interference_token(token);
        t_interference = interf_pair.first;
        id_interference = interf_pair.second;
        if (t_interference)
            c_print("Robot in interferenza: ", ID_ROBOT, red, P);

        if (goal_complete)
        {
            c_print("before OnGoal()", magenta);
            onGoalComplete(token);
            resend_goal_count = 0;
        }
    }

    usleep(30000);
    token_pub.publish(token);
    ros::spinOnce();

    if (token.END_SIMULATION)
    {
        end_simulation = true;
    }
} // token_callback()
// -----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    spartagent::SPartAgent SPA;
    SPA.init(argc, argv);
    c_print("@ SPART", green);
    sleep(3);
    SPA.run();
    return 0;
}