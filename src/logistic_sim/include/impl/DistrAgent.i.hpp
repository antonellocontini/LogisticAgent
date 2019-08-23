#pragma once

namespace distragent
{

void DistrAgent::init(int argc, char **argv)
{
    Agent::init(argc, argv);
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

    //  init_agent3();

    // c_print("RequestTask", green);
    // request_Task();

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    while (ros::ok())
    {

        //  cosa ho globale??? mi serve tempo velocita e posizione
        float x, y, t;
        getRobotPose(ID_ROBOT, x, y, t);
        std::cout << "x: " << xPos[ID_ROBOT] << " y: " << yPos[ID_ROBOT] << " th: " << thetaPos[ID_ROBOT] << "\n"
                  << "lastPos: x= " << lastXpose << " ,y= " << lastYpose << "\n";

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

    // aggiorniamo condizioni destinazione
    if (go_home && current_vertex == initial_vertex)
    {
        need_task = true;
    }
    else if (current_vertex == 6)
    {
        reached_pickup = true;
    }
    else if (current_vertex == current_task.DSTS[0] && reached_pickup)
    {
        need_task = true;
        reached_pickup = false;
    }

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
    else if (!reached_pickup)
    {
        tp_dijkstra(current_vertex, 6, path, path_length);
    }
    else
    {
        tp_dijkstra(current_vertex, current_task.DSTS[0], path, path_length);
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

void DistrAgent::init_tw_map()
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

void DistrAgent::tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path)
{
    const int PENALTY = 500;

    vertex web_copy[dimension];
    for (int i = 0; i < dimension; i++)
    {
        // copia della struttura
        // web_copy[i] = vertex_web[i];
        web_copy[i].id = vertex_web[i].id;
        web_copy[i].num_neigh = vertex_web[i].num_neigh;
        web_copy[i].x = vertex_web[i].x;
        web_copy[i].y = vertex_web[i].y;
        memcpy(web_copy[i].id_neigh, vertex_web[i].id_neigh, sizeof(uint) * 8);
        memcpy(web_copy[i].cost, vertex_web[i].cost, sizeof(uint) * 8);
        memcpy(web_copy[i].cost_m, vertex_web[i].cost_m, sizeof(float) * 8);
        memcpy(web_copy[i].visited, vertex_web[i].visited, sizeof(bool) * 8);
        for (int j = 0; j < web_copy[i].num_neigh; j++)
        {
            memcpy(web_copy[i].dir[j], vertex_web[i].dir[j], sizeof(char) * 3);
        }

        // incremento il peso degli archi occupati
        for (int j = 0; j < web_copy[i].num_neigh; j++)
        {
            uint from = web_copy[i].id;
            uint to = web_copy[i].id_neigh[j];
            web_copy[i].cost[j] += PENALTY * token_weight_map[from][to];
        }
    }

    // calcolo il percorso con il nuovo grafo
    dijkstra(source, destination, shortest_path, elem_s_path, web_copy, dimension);
}

uint DistrAgent::compute_id_path(std::vector<logistic_sim::Task> m)
{
    uint res = 0;
    std::vector<uint> d;
    d.clear();
    for (auto i = 0; i < m.size(); i++)
    {
        d = m[i].DSTS;
        
    }
    sort(d.begin(), d.end());
    d.erase(unique(d.begin(), d.end()), d.end());

    if (d.size() == 1)
    {
        if (d[0] == 11)
            res = 1;
        else if (d[0] == 16)
            res = 2;
        else
            res = 3;
    }
    else if (d.size() == 2)
    {
        if ((d[0] == 11) && (d[1] == 16))
        {
            res = 4;
        }
        else if ((d[0] == 11) && (d[1] == 21))
        {
            res = 5;
        }
        else
        {
            res = 6;
        }
    }
    else
    {
        res = 7;
    }

    return res;
}

int DistrAgent::compute_cost_of_route(std::vector<uint> route)
{
    int custo_final = 0;
    for (int i = 1; i < route.size(); i++)
    {
        int anterior = route[i - 1];
        int proximo = route[i];

        for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
        {
            if (vertex_web[anterior].id_neigh[j] == proximo)
            {
                custo_final += vertex_web[anterior].cost[j];
                break;
            }
        }
    }
    return custo_final;
}

void DistrAgent::compute_travell(uint id_path, logistic_sim::Mission m)
{
    switch (id_path)
    {
    case 1:
    {
        for (auto i = 0; i < 8; i++)
        {
            m.ROUTE.push_back(p_11[i]);
        }

        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 2:
    {
        for (auto i = 0; i < 12; i++)
        {
            m.ROUTE.push_back(p_16[i]);
        }
        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 3:
    {
        for (auto i = 0; i < 16; i++)
        {
            m.ROUTE.push_back(p_21[i]);
        }
        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 4:
    {
        for (auto i = 0; i < 14; i++)
        {
            m.ROUTE.push_back(p_11_16[i]);
        }
        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;

    case 5:
    {
        for (auto i = 0; i < 18; i++)
        {
            m.ROUTE.push_back(p_11_21[i]);
        }
        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 6:
    {
        for (auto i = 0; i < 18; i++)
        {
            m.ROUTE.push_back(p_16_21[i]);
        }
        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 7:
    {
        for (auto i = 0; i < 20; i++)
        {
            m.ROUTE.push_back(p_11_16_21[i]);
        }
        m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
        m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    default:
    {
        c_print("ERR", red);
    }
    break;
    }
}

logistic_sim::Mission DistrAgent::coalition_formation(logistic_sim::Token token)
{
    auto tmp_CAPACITY = CAPACITY;

    auto size_tasks_set = token.MISSION.size();

    int id = 0;
    // init
    for (auto i = 0; i < size_tasks_set; i++)
    {

        logistic_sim::Mission m;
        m.ID = id;
        id++;
        m.MISSION.push_back(token.MISSION[i]);
        m.TOT_DEMAND = token.MISSION[i].DEMAND;
        uint id_path = compute_id_path(token.MISSION);
        compute_travell(id_path, m);
        if (coalition.empty())
        {
            c_print("[W] coalition vuota!", yellow, P);
        }
        coalition.push_back(m);
    }
    // doppio ciclo
    logistic_sim::Mission mission1;
    logistic_sim::Mission mission2;
    auto new_id = coalition.size();
    for (auto i = 0; i < coalition.size(); i++)
    {
        mission1 = coalition[i];
        for (auto j = 0; j < coalition.size(); j++)
        {
            mission2 = coalition[j];
            if (mission1.ID != mission2.ID)
            {
                auto dEMAND = mission1.TOT_DEMAND + mission2.TOT_DEMAND;
                if (dEMAND <= tmp_CAPACITY)
                {
                    logistic_sim::Mission mission3;
                    mission3.ID = new_id;
                    new_id++;
                    mission3.TOT_DEMAND = dEMAND;
                    for (auto y = 0; y < mission1.MISSION.size(); y++)
                        mission3.MISSION.push_back(mission1.MISSION[y]);
                    for (auto w = 0; w < mission2.MISSION.size(); w++)
                        mission3.MISSION.push_back(mission2.MISSION[w]);
                    uint id_path = compute_id_path(mission3.MISSION);
                    compute_travell(id_path, mission3);

                    if ((mission3.V - mission1.V - mission2.V) < 0)
                    {
                        cout << " mission3: "<< mission3.ROUTE.size() << "\n";
                        for (auto i = 0; i  < mission3.ROUTE.size(); i++)
                        {
                            cout << mission3.ROUTE[i]<< " ";
                        }
                        cout << " \n";
                        // mission3 e' la prima coalizione che mi va bene
                        // devo rimuovere dal token i task della coalizione e rimandare il token
                    }
                }
            }
        }
    }
    //
}

void DistrAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{

    // ricevo il token ricevo il task set computo la CF migliore la assegno e toglo i task che la compongono.

    // se non è per me termino
    if (msg->ID_RECEIVER != ID_ROBOT)
        return;

    logistic_sim::Token token;
    token = *msg;

    logistic_sim::Mission m = coalition_formation(token);


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
        token.CURR_VERTEX.push_back(current_vertex);
        token.NEXT_VERTEX.push_back(current_vertex);

        initialize = false;
    }
    else
    {
        if (need_task)
        {
            if (!msg->MISSION.empty())
            {
                // prendo la prima missione
                auto t = msg->MISSION.back();
                token.MISSION.pop_back();
                token.ASSIGNED_MISSION.push_back(t);
                current_task = t;
            }
            else
            {
                go_home = true;
            }
            need_task = false;
            goal_complete = true;
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

                c_print("[DEBUG]\ttw_map updated: [", src, ",", dst, "]", yellow);
                //svaforisco la mia direzione
                token_weight_map[src][dst]++;
                //sfavorisco la direzione inversa
                token_weight_map[dst][src] += 3;
            }
        }

        // metto nel token quale arco sto occupando
        token.CURR_VERTEX[ID_ROBOT] = current_vertex;
        token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
    }

    usleep(200000);
    token_pub.publish(token);
    ros::spinOnce();
} // token_callback()

} // namespace distragent