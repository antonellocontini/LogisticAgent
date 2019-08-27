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

// mi basta la prima coalizione buona e poi posso uscire dal ciclo
logistic_sim::Mission DistrAgent::coalition_formation(logistic_sim::Token token)
{
    logistic_sim::Mission M;
    // in coalition ho tutte le missioni cerco la combinazione che mi soddisfa e passo il token
    auto size_tasks_set = token.TASK.size();

    int id = 0;
    // init
    coalition.clear();
    for (auto i = 0; i < size_tasks_set; i++)
    {
        logistic_sim::Mission m;
        m.ID = id;
        id++;
        m.MISSION.push_back(token.TASK[i]);
        m.TOT_DEMAND = token.TASK[i].DEMAND;
        uint id_path = compute_id_path(token.TASK);
        compute_travell(id_path, m);
        if (coalition.empty())
        {
            c_print("[W] coalition vuota!", yellow, P);
        }
        coalition.push_back(m);
    }
    c_print("inizializzazione della coalizione", green, P);
    // doppio ciclo
    logistic_sim::Mission mission1;
    logistic_sim::Mission mission2;
    auto new_id = coalition.size();
    for (auto i = 0; i < coalition.size(); i++)
    {
        mission1 = coalition[i];
        if (mission1.TOT_DEMAND == tmp_CAPACITY)
        {
            // allora la piazzo subito
            M = mission1;
            break; //esco e ciao
        }
        else if (mission1.TOT_DEMAND < tmp_CAPACITY)
        {
            // ciclo con altro elemento per vedere se soddisfo
            for (auto j = 0; j < coalition.size(); j++)
            {
                mission2 = coalition[j];
                if (mission1.ID != mission2.ID)
                {
                    auto dEMAND = mission1.TOT_DEMAND + mission2.TOT_DEMAND;
                    if (dEMAND <= tmp_CAPACITY)
                    {
                        // c_print("Coalizione trovata", yellow, P);
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

                        // minimizzazione
                        if ((mission3.V - mission1.V - mission2.V) < 0)
                        {
                            c_print("questa va da dio CANE", green,P);
                            // eliminazione degli elemnti che compongono la missione
                            // operator == TODO FIX
                            // coalition.erase(find(coalition.begin(), coalition.end(), cmp_Mission(mission1, mission2)));
                            // coalition.erase(find(coalition.begin(), coalition.end(), mission2));

                            // coalition.push_back(mission3);

                            c_print("[W]: mission3", yellow, P);
                            cout << " mission3: " << mission3.ROUTE.size() << "\n";
                            for (auto i = 0; i < mission3.ROUTE.size(); i++)
                            {
                                cout << mission3.ROUTE[i] << " ";
                            }
                            cout << " \n";

                            M = mission3;

                            break;
                            // mission3 e' la prima coalizione che mi va bene
                            // devo rimuovere dal token i task della coalizione e rimandare il token
                        }
                        else
                        {
                            // delete mission3;
                        }
                    }
                }
            }
        }
    }
    //inserisco tutto su un vettore e poi prendo la coalizione migliore?
    // oppure prendo la prima coalizione che mi soddisfa (tesi coalizione migliore)
    return M;
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

            c_print("##################################", magenta, P);
            logistic_sim::Mission m = coalition_formation(token);

            cout << "missione finale dopo coalizione: " << m.ID << m.TOT_DEMAND << "\n";

            if (!msg->MISSION.empty())
            {
                // qua asegno la missione 
                // prendo la prima missione
                auto t = msg->TASK.back();
                token.TASK.pop_back();
                token.ASSIGNED_TASK.push_back(t);
                current_task = t;
                // qua ci metto il task che mi serve dal token io non ho il sisngolo task ma la missione composta da task
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