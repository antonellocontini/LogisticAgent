#pragma once

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
            web_copy[i].cost[j] += PENALTY * token_weight_map[from][to];
        }
    }

    // calcolo il percorso con il nuovo grafo
    dijkstra(source, destination, shortest_path, elem_s_path, web_copy, dimension);
}

uint DistrAgent::compute_id_path(logistic_sim::Mission &m)
{
    uint res = 0;
    std::vector<uint> d;
    d.clear();

    d = m.DSTS;

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

// mi basta la prima coalizione buona e poi posso uscire dal ciclo
logistic_sim::Mission DistrAgent::coalition_formation(logistic_sim::Token &token)
{
    std::vector<t_coalition> coalitions;
    t_coalition best_coalition; //la migliore finore 
    static int id = 500;
    for(auto it = token.MISSION.begin(); it != token.MISSION.end(); it++)
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
            c_print("iterazione n ", count, yellow, P);
            if (!(c1.second == c2.second))
            {
                auto tmp_DEMAND = c1.second.TOT_DEMAND + c2.second.TOT_DEMAND;
                if (tmp_DEMAND <= tmp_CAPACITY)
                {
                    c_print("sono dentro", yellow, P);
                    logistic_sim::Mission m;    // possibile coalizione
                    m.ID = id++;

                    copy(c1.second.DSTS.begin(), c1.second.DSTS.end(), back_inserter(m.DSTS));
                    // copy(c2.DSTS.begin(), (*jt).DSTS.end(), back_inserter(m.DSTS));

                    copy(c1.second.DEMANDS.begin(), c1.second.DEMANDS.end(), back_inserter(m.DEMANDS));
                    // copy((*jt).DEMANDS.begin(), (*jt).DEMANDS.end(), back_inserter(m.DEMANDS));

                    logistic_sim::Mission &m2 = c2.second;    // seconda missione da unire a m
                    for (int zt = 0; zt < m2.DSTS.size(); zt++)
                    {
                        bool found = false;
                        for (int xt = 0; xt < m.DSTS.size() && !found; xt++)
                        {
                            if (m2.DSTS[zt] == m.DSTS[xt])
                            {
                                m.DEMANDS[xt] += m2.DEMANDS[zt];
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
                    c_print("M1 ID ", m1.ID, red, P);
                    for(int i=0; i<m1.DSTS.size(); i++)
                    {
                        c_print("M1 DSTS ", m1.DSTS[i], "\tDEMANDS ", m1.DEMANDS[i], yellow, P);
                    }
                    c_print("M2 ID ", m2.ID, red, P);
                    for(int i=0; i<m2.DSTS.size(); i++)
                    {
                        c_print("M2 DSTS ", m2.DSTS[i], "\tDEMANDS ", m2.DEMANDS[i], yellow, P);
                    }
                    c_print("M_TOT ID ", m.ID, red, P);
                    for(int i=0; i<m.DSTS.size(); i++)
                    {
                        c_print("M_TOT DSTS ", m.DSTS[i], "\tDEMANDS ", m.DEMANDS[i], yellow, P);
                    }

                    m.TOT_DEMAND = std::accumulate(m.DEMANDS.begin(), m.DEMANDS.end(), 0);
                    c_print("capacita: m1 ", c1.second.TOT_DEMAND, " m2 ", m2.TOT_DEMAND, " totale ", m.TOT_DEMAND, yellow, P);

                    auto id_path = compute_id_path(m);

                    compute_travell(id_path, m);

                    double coal_V = m.V;
                    double first_V = c1.second.V;
                    double second_V = c2.second.V;
                    double res = (coal_V - (first_V + second_V));
                    c_print("coal_V ", coal_V, "\tfirst_V ", first_V, "\tsecond_V", second_V, yellow, P);
                    
                    c_print("before", red, P);
                    for(int j=0; j<coalitions.size(); j++)
                    {
                        logistic_sim::Mission &temp_m = coalitions[j].second;
                        for(int i=0; i<temp_m.DSTS.size(); i++)
                        {
                            c_print("ID ", temp_m.ID, "\tDSTS ", temp_m.DSTS[i], "\tDEMANDS ", temp_m.DEMANDS[i], yellow, P);
                        }
                    }
                    
                    if (res < 0)
                    {
                        t_coalition tmp_coalition;
                        tmp_coalition.first.insert(tmp_coalition.first.end(), c1.first.begin(), c1.first.end());
                        tmp_coalition.first.insert(tmp_coalition.first.end(), c2.first.begin(), c2.first.end());
                        tmp_coalition.second = m;
                        coalitions.push_back(tmp_coalition);
                        coalitions.erase(find(coalitions.begin(), coalitions.end(), c1));
                        coalitions.erase(find(coalitions.begin(), coalitions.end(), c2));
                        c_print("after", red, P);
                        for(int j=0; j<coalitions.size(); j++)
                        {
                            logistic_sim::Mission &temp_m = coalitions[j].second;
                            for(int i=0; i<temp_m.DSTS.size(); i++)
                            {
                                c_print("ID ", temp_m.ID, "\tDSTS ", temp_m.DSTS[i], "\tDEMANDS ", temp_m.DEMANDS[i], yellow, P);
                            }
                        }
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
             c_print("siamo uguali",yellow,P);
            }

            c_print("\n\n", P);
            count++;
        }
    }

    c_print("size della colaitions: ", coalitions.size(), green);

    best_coalition = *coalitions.begin();

    for (auto it = coalitions.begin(); it != coalitions.end(); it++)
    {
        if ((*it).second.V < best_coalition.second.V)
        {
            best_coalition = (*it);
        }
    }

    // aggirnamento del token 
    for (auto i = 0 ; i < best_coalition.first.size(); i++)
    {
        token.MISSION.erase(find(token.MISSION.begin(), token.MISSION.end(), best_coalition.first[i]));
    }

    return best_coalition.second;
}

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
        token.CURR_VERTEX.push_back(current_vertex);
        token.NEXT_VERTEX.push_back(current_vertex);

        initialize = false;
    }
    else
    {
        if (need_task)
        {
            if(token.MISSION.size() == 0)
            {
                go_home = true;
                need_task = false;
            }
            else
            {
                c_print("[DEBUG]\tsize before coalition: ", token.MISSION.size(), "\tcapacity: ", tmp_CAPACITY, yellow);
                c_print("##################################", magenta, P);
                current_mission = coalition_formation(token);
                c_print("[DEBUG]\tsize after coalition: ", token.MISSION.size(), yellow);

                if (current_mission.ID != 66)
                {
                    current_mission.PICKUP = true;

                    need_task = false;
                    goal_complete = true;
                }
                else
                {
                    c_print("[WARN]\tNon posso prendere task, capacity: ", tmp_CAPACITY, red);
                }
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

                // c_print("[DEBUG]\ttw_map updated: [", src, ",", dst, "]", yellow);
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