namespace distragent
{

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
            web_copy[i].cost[j] *= token_weight_map[from][to];
        }
    }

    // calcolo il percorso con il nuovo grafo
    dijkstra(source, destination, shortest_path, elem_s_path, web_copy, dimension);
}

uint DistrAgent::compute_id_path(logistic_sim::Mission &m)
{
    uint res = 0;
    std::vector<uint> d;

    d = m.DSTS;

    sort(d.begin(), d.end());
    d.erase(unique(d.begin(), d.end()), d.end());

    if (d.size() == 1)
    {
        if (d[0] == 18)
            res = 1;
        else if (d[0] == 23)
            res = 2;
        else
            res = 3;
    }
    else if (d.size() == 2)
    {
        if ((d[0] == 18) && (d[1] == 23))
        {
            res = 4;
        }
        else if ((d[0] == 18) && (d[1] == 28))
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

void DistrAgent::compute_travell(uint id_path, logistic_sim::Mission &m)
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

void DistrAgent::print_coalition(const t_coalition &coalition)
{
    auto tasks = coalition.first;
    auto mission = coalition.second;
    c_print("Mission id: ", mission.ID, green, P);
    // std::cout << "pd: " << mission.PATH_DISTANCE << "\n";
    // std::cout << "td: " << mission.TOT_DEMAND << "\n";
    std::cout << " V: " << mission.V << "\n";
    // auto size_dsts = mission.DSTS.size();
    // std::cout << "dsts"
    //           << "\n";
    // for (int i = 0; i < size_dsts; i++)
    // {
    //     std::cout << mission.DSTS[i] << " ";
    // }
    // std::cout << "\n";

    // auto size_boh = mission.DEMANDS.size();
    // std::cout << "demands"
    //           << "\n";
    // for (int i = 0; i < size_boh; i++)
    // {
    //     std::cout << mission.DEMANDS[i] << " ";
    // }
    // std::cout << "\n";

    // auto size_route = mission.ROUTE.size();
    // std::cout << "route"
    //           << "\n";
    // for (int i = 0; i < size_route; i++)
    // {
    //     std::cout << mission.ROUTE[i] << " ";
    // }
    // std::cout << "\n";

    // c_print("tasks", magenta, P);

    // auto size_tasks = tasks.size();

    // for (auto i = 0; i < size_tasks; i++)
    // {
    //     auto t = tasks[i];
    //     c_print("task id: ", t.ID, magenta, P);
    //     std::cout << "pd: " << t.PATH_DISTANCE << "\n";
    //     std::cout << "td: " << t.TOT_DEMAND << "\n";
    //     std::cout << " V: " << t.V << "\n";
    //     auto size_dsts = t.DSTS.size();
    //     std::cout << "dsts"
    //               << "\n";
    //     for (int i = 0; i < size_dsts; i++)
    //     {
    //         std::cout << t.DSTS[i] << " ";
    //     }
    //     std::cout << "\n";

    //     auto size_boh = t.DEMANDS.size();
    //     std::cout << "demands"
    //               << "\n";
    //     for (int i = 0; i < size_boh; i++)
    //     {
    //         std::cout << t.DEMANDS[i] << " ";
    //     }
    //     std::cout << "\n";

    //     auto size_route = t.ROUTE.size();
    //     std::cout << "route"
    //               << "\n";
    //     for (int i = 0; i < size_route; i++)
    //     {
    //         std::cout << t.ROUTE[i] << " ";
    //     }
    //     std::cout << "\n";
    // }

    c_print("fine mission id: ", mission.ID, red, P);
}

logistic_sim::Mission DistrAgent::coalition_formation(logistic_sim::Token &token)
{
    std::vector<t_coalition> coalitions;
    c_print("tmp_CAPACITY: ", tmp_CAPACITY, green, P);
    t_coalition best_coalition; //la migliore finore
    static int id = 500;
    for (auto it = token.MISSION.begin(); it != token.MISSION.end(); it++)
    {
        t_coalition temp;
        temp.second = *it;
        temp.first.push_back(*it);
        coalitions.push_back(temp);
    }

    int count = 1;
    for (auto it = 0; it < coalitions.size(); it++)
    {
        t_coalition &c1 = coalitions[it];
        for (auto jt = 0; jt < coalitions.size(); jt++)
        {
            t_coalition &c2 = coalitions[jt];
            // c_print("iterazione n ", count, yellow, P);
            if (!(c1.second == c2.second))
            {
                auto tmp_DEMAND = c1.second.TOT_DEMAND + c2.second.TOT_DEMAND;
                if (tmp_DEMAND <= tmp_CAPACITY)
                {
                    // c_print("sono dentro", yellow, P);
                    logistic_sim::Mission m; // possibile coalizione
                    m.ID = id++;

                    copy(c1.second.DSTS.begin(), c1.second.DSTS.end(), back_inserter(m.DSTS));
                    // copy(c2.DSTS.begin(), (*jt).DSTS.end(), back_inserter(m.DSTS));

                    copy(c1.second.DEMANDS.begin(), c1.second.DEMANDS.end(), back_inserter(m.DEMANDS));
                    // copy((*jt).DEMANDS.begin(), (*jt).DEMANDS.end(), back_inserter(m.DEMANDS));

                    logistic_sim::Mission &m2 = c2.second; // seconda missione da unire a m
                    for (int zt = 0; zt < m2.DSTS.size(); zt++)
                    {
                        bool found = false;
                        for (int xt = 0; xt < m.DSTS.size() && !found; xt++)
                        {
                            if (m2.DSTS[zt] == m.DSTS[xt])
                            {
                                m.DEMANDS.insert(m.DEMANDS.begin() + xt, m2.DEMANDS[zt]);
                                m.DSTS.insert(m.DSTS.begin() + xt, m2.DSTS[zt]);
                                found = true;
                            }
                        }

                        if (!found)
                        {
                            m.DSTS.push_back(m2.DSTS[zt]);
                            m.DEMANDS.push_back(m2.DEMANDS[zt]);
                        }
                    }

                    // stampe debug
                    logistic_sim::Mission &m1 = c1.second;
                    // c_print("M1 ID ", m1.ID, red, P);
                    // for (int i = 0; i < m1.DSTS.size(); i++)
                    // {
                    //     c_print("M1 DSTS ", m1.DSTS[i], "\tDEMANDS ", m1.DEMANDS[i], yellow, P);
                    // }
                    // c_print("M2 ID ", m2.ID, red, P);
                    // for (int i = 0; i < m2.DSTS.size(); i++)
                    // {
                    //     c_print("M2 DSTS ", m2.DSTS[i], "\tDEMANDS ", m2.DEMANDS[i], yellow, P);
                    // }
                    // c_print("M_TOT ID ", m.ID, red, P);
                    // for (int i = 0; i < m.DSTS.size(); i++)
                    // {
                    //     c_print("M_TOT DSTS ", m.DSTS[i], "\tDEMANDS ", m.DEMANDS[i], yellow, P);
                    // }

                    m.TOT_DEMAND = std::accumulate(m.DEMANDS.begin(), m.DEMANDS.end(), 0);
                    // c_print("capacita: m1 ", c1.second.TOT_DEMAND, " m2 ", m2.TOT_DEMAND, " totale ", m.TOT_DEMAND, yellow, P);

                    auto id_path = compute_id_path(m);

                    compute_travell(id_path, m);

                    double coal_V = m.V;
                    double first_V = c1.second.V;
                    double second_V = c2.second.V;
                    double res = (coal_V - (first_V + second_V));
                    // c_print("coal_V ", coal_V, "\tfirst_V ", first_V, "\tsecond_V", second_V, yellow, P);

                    // c_print("before", red, P);
                    // for (int j = 0; j < coalitions.size(); j++)
                    // {
                    //     logistic_sim::Mission &temp_m = coalitions[j].second;
                    //     for (int i = 0; i < temp_m.DSTS.size(); i++)
                    //     {
                    //         c_print("ID ", temp_m.ID, "\tDSTS ", temp_m.DSTS[i], "\tDEMANDS ", temp_m.DEMANDS[i], yellow, P);
                    //     }
                    // }

                    if (res < 0)
                    {
                        t_coalition tmp_coalition;
                        tmp_coalition.first.insert(tmp_coalition.first.end(), c1.first.begin(), c1.first.end());
                        tmp_coalition.first.insert(tmp_coalition.first.end(), c2.first.begin(), c2.first.end());
                        tmp_coalition.second = m;
                        coalitions.push_back(tmp_coalition);
                        coalitions.erase(find(coalitions.begin(), coalitions.end(), c1));
                        coalitions.erase(find(coalitions.begin(), coalitions.end(), c2));
                        // c_print("after", red, P);
                        // for (int j = 0; j < coalitions.size(); j++)
                        // {
                        //     logistic_sim::Mission &temp_m = coalitions[j].second;
                        //     for (int i = 0; i < temp_m.DSTS.size(); i++)
                        //     {
                        //         c_print("ID ", temp_m.ID, "\tDSTS ", temp_m.DSTS[i], "\tDEMANDS ", temp_m.DEMANDS[i], yellow, P);
                        //     }
                        // }
                        break;
                    }
                }
                else
                {
                    c_print("la capacita e' troppo alta", tmp_DEMAND, yellow, P);
                }
            }
            else
            {
                // c_print("siamo uguali", yellow, P);
            }

            // c_print("\n\n", P);
            count++;
        }
    }

    c_print("size della coalitions: ", coalitions.size(), green);

    std::sort(coalitions.begin(), coalitions.end(), less_V());

    for (auto i = 0; i < coalitions.size(); i++)
    {
        print_coalition(coalitions[i]);
    }

    auto coit = coalitions.begin();
    for (; coit != coalitions.end(); coit++)
    {
        best_coalition = *coit;

        auto dst_cm = best_coalition.second.DSTS.begin();
        for (auto i = 0; i < token.CURR_DST.size() && dst_cm != best_coalition.second.DSTS.end(); i++)
        {
            if (*dst_cm == token.CURR_DST[i] && i != ID_ROBOT)
            {
                dst_cm++;
                i = -1;
            }
        }

        if (dst_cm != best_coalition.second.DSTS.end())
        {
            auto temp = *dst_cm;
            best_coalition.second.DSTS.erase(dst_cm);
            best_coalition.second.DSTS.insert(best_coalition.second.DSTS.begin(), temp);
            break;
        }
    }

    if (coit == coalitions.end())
    {
        c_print("Robot ", ID_ROBOT, " Nessuna missione conflict-free", red);
        // logistic_sim::Mission safe_mission;
        // safe_mission.ID = 123;
        // safe_mission.DSTS.push_back(26); //indice del nodo_safe
        // safe_mission.DEMANDS.push_back(0); //fittizia
        // safe_mission.TOT_DEMAND = 0;
        // safe_mission.PICKUP = false;
        // best_coalition.second = safe_mission; //aggiorno la current
        // best_coalition.first.clear();
        best_coalition = coalitions.front();
    }

    best_coalition.second.PICKUP = true;
    // aggirnamento del token
    for (auto i = 0; i < best_coalition.first.size(); i++)
    {
        token.MISSION.erase(find(token.MISSION.begin(), token.MISSION.end(), best_coalition.first[i]));
    }

    return best_coalition.second;
}

// il primo campo è il tipo di interferenza
// il secondo è il robot che mi ha causato l'interferenza
std::pair<int,int> DistrAgent::check_interference_token(logistic_sim::Token &token)
{
    int i;
    double dist_quad;

    if (ros::Time::now().toSec() - last_interference < 10) // seconds
    {
        return std::pair<int,int>(t_interference, id_interference);    // false if within 10 seconds from the last one
    }

    /* Poderei usar TEAMSIZE para afinar */
    // ID_ROBOT
    for (i = 0; i < TEAM_SIZE; i++)
    { //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)

        dist_quad = (xPos[i] - xPos[ID_ROBOT]) * (xPos[i] - xPos[ID_ROBOT]) + (yPos[i] - yPos[ID_ROBOT]) * (yPos[i] - yPos[ID_ROBOT]);

        if (i!= ID_ROBOT && dist_quad <= INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE)
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
            double other_distance = (xPos[i] - other_x_dst) * (xPos[i] - other_x_dst) + (yPos[i] - other_y_dst) * (yPos[i] - other_y_dst);
            double my_distance = (xPos[ID_ROBOT] - x_dst) * (xPos[ID_ROBOT] - x_dst) + (yPos[ID_ROBOT] - y_dst) * (yPos[ID_ROBOT] - y_dst);
            // c_print("my_distance ", my_distance, "\tother_distance ", other_distance, yellow);
            last_interference = ros::Time::now().toSec();
            if (my_metric < other_metric)
            {
                token.INTERFERENCE_COUNTER[ID_ROBOT]++;
                return std::pair<int, int>(1, i);
            }
            else
            {
                c_print("[DEBUG]\tDovrebbe andare ROBOT_ID: ", i, " in interferenza", red, P);
            }
            
        }
    }
    return std::pair<int, int>(0, 0);
}

void DistrAgent::onGoalComplete(logistic_sim::Token &token)
{
    if (next_vertex > -1)
    {
        int dist = compute_cost_of_route({current_vertex, (uint)next_vertex});
        token.MISSION_CURRENT_DISTANCE[ID_ROBOT] += (float)dist;
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
        token.TOTAL_DISTANCE[ID_ROBOT] += token.MISSION_CURRENT_DISTANCE[ID_ROBOT];
        token.MISSION_CURRENT_DISTANCE[ID_ROBOT] = 0.0f;
        need_task = true;
    }
    else if (go_src() && current_vertex == src_vertex)
    {
        current_mission.PICKUP = false;
        tmp_CAPACITY -= current_mission.TOT_DEMAND;
        token.TOTAL_DISTANCE[ID_ROBOT] += token.MISSION_CURRENT_DISTANCE[ID_ROBOT];
        token.MISSION_CURRENT_DISTANCE[ID_ROBOT] = 0.0f;
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
            token.MISSIONS_COMPLETED[ID_ROBOT]++;
            need_task = true;
        }

        token.TASKS_COMPLETED[ID_ROBOT]++;
        token.TOTAL_DISTANCE[ID_ROBOT] += token.MISSION_CURRENT_DISTANCE[ID_ROBOT];
        token.MISSION_CURRENT_DISTANCE[ID_ROBOT] = 0.0f;

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
    next_vertex = compute_next_vertex(token);

    c_print("   @ compute_next_vertex: ", next_vertex, green);

    send_goal_reached(); // Send TARGET to monitor

    send_results(); // Algorithm specific function

    // Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    // sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    sendGoal(next_vertex); // send to move_base

    goal_complete = false;
}

int DistrAgent::compute_next_vertex(logistic_sim::Token &token)
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
        tp_dijkstra(current_vertex, src_vertex, path, path_length); //id src 13 prima 6
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

void DistrAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
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

} // namespace distragent