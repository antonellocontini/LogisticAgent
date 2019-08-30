#pragma once

namespace tokenagent
{

void TokenAgent::init(int argc, char **argv)
{
    Agent::init(argc, argv);
    ros::NodeHandle nh;

    token_pub = nh.advertise<logistic_sim::Token>("token", 1);

    token_sub = nh.subscribe<logistic_sim::Token>("token", 20, boost::bind(&TokenAgent::token_callback, this, _1));

    init_tw_map();
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

    ros::Rate loop_rate(30); // 0.033 seconds or 30Hz

    // TEST: attesa di qualche secondo dalla partenza del precedente
    int wait_time = 13 * ID_ROBOT;
    c_print("[DEBUG]\tAttendo ", wait_time, " secondi...", yellow, P);
    sleep(wait_time);
    c_print("[DEBUG]\tParto", P);
    init_wait_done = true;

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
                // do_interference_behavior();
                // invece di eseguire il comportamento di interferenza
                // provo a dire al robot di andare al vertice da dove è arrivato
                uint temp = next_vertex;
                next_vertex = current_vertex;
                current_vertex = temp;
                // se non setto interference a false questo ramo viene eseguito un paio
                // di volte poichè il token deve completare il giro prima che la variabile
                // interference venga calcolata
                interference = false;
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

   // aggiorniamo condizioni destinazione
    if(go_home && current_vertex == initial_vertex)
    {
        need_task = true;
    }
    else if (current_vertex == 6)
    {
        reached_pickup = true;
        sleep(2);
    }
    else if(current_vertex == current_mission.DSTS[0] && reached_pickup)
    {
        need_task = true;
        reached_pickup = false;
        sleep(2);
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

    // setto lo start_time al tempo attuale
    goal_start_time = ros::Time::now();

    goal_complete = false;
}


int TokenAgent::compute_next_vertex()
{
    int vertex;

    // alloco un vettore per il percorso
    int path[dimension];
    uint path_length;

    if(go_home)
    {
        c_print("[DEBUG]\tgoing_home...\tinitial_vertex: ", initial_vertex, yellow, P);
        tp_dijkstra(current_vertex, initial_vertex, path, path_length);
    }
    else if (!reached_pickup)
    {
        tp_dijkstra(current_vertex, 6, path, path_length);
    }
    else
    {
        tp_dijkstra(current_vertex, current_mission.DSTS[0], path, path_length);
    }

    c_print("[DEBUG]\tpath_length: ", path_length, "\tpath:", yellow, P);
    for(int i=0; i<path_length; i++)
    {
        c_print("\t\t", path[i], yellow, P);
    }

    if (path_length > 1)
    {
        vertex = path[1];   //il primo vertice è quello di partenza, ritorno il secondo
    }
    else
    {
        vertex = current_vertex;
    }
    

    return vertex;
} // compute_next_vertex()

void TokenAgent::init_tw_map()
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


void TokenAgent::tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path)
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
            //web_copy[i].cost[j] += PENALTY*token_weight_map[from][to];
            web_copy[i].cost[j] *= token_weight_map[from][to];
        }
    }

    // calcolo il percorso con il nuovo grafo
    dijkstra( source, destination, shortest_path, elem_s_path, web_copy, dimension );
}

void TokenAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
    // se non è per me termino
    if(msg->ID_RECEIVER != ID_ROBOT)
        return;

    logistic_sim::Token token;
    token = *msg;

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

        c_print("[DEBUG]\tinit_current_vertex: ", current_vertex, yellow, P);

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
        // inizializzo con valore non valido(non esiste id vertice maggiore di dimension)
        token.CURR_DST.push_back(dimension + 1);

        initialize = false;
    }
    else
    {
        if(need_task)
        {
            //if(!msg->TASK.empty())
            if(!token.MISSION.empty())
            {
                int m_size = token.MISSION.size();
                // prendo la prima missione
                // auto t = msg->TASK.back();
                // token.TASK.pop_back();
                // token.ASSIGNED_MISSION.push_back(t);

                // se possibile prendo un task con destinazione diversa da quelli in corso
                // per farlo ciclo tra tutti i task disponibili finchè non ne trovo uno compatibile
                c_print("[DEBUG]\tScelta task...", yellow, P);
                auto it_m = token.MISSION.begin();
                bool good = false;
                for( ; it_m != token.MISSION.end() && !good; it_m++)
                {
                    good = true;
                    for(auto it_dst = msg->CURR_DST.begin(); it_dst != msg->CURR_DST.end(); it_dst++)
                    {
                        if(*it_dst == (*it_m).DSTS[0])
                        {
                            good = false;
                        }
                    }
                }

                c_print("[DEBUG]\tConflict-free=", good, yellow, P);
                logistic_sim::Mission t;
                // ho trovato un task adeguato, lo prendo dall'iteratore
                if(good)
                {
                    it_m--; // alla fine del ciclo precedente l'iteratore è andato oltre il task selezionato
                    t = *it_m;
                    c_print("[DEBUG]\tDST=", t.DSTS[0], yellow, P);
                    token.MISSION.erase(it_m);
                }
                else    // non esiste un task che non vada in conflitto, prendo l'ultimo della lista
                {
                    t = token.MISSION.back();
                    token.MISSION.pop_back();
                }
                
                c_print("[DEBUG]\tTask rimosso dalla lista", yellow, P);
                token.ASSIGNED_MISSION.push_back(t);
                token.CURR_DST[ID_ROBOT] = t.DSTS[0];
                current_mission = t;
                c_print("[DEBUG]\tTask assegnato nel token", yellow, P);

                if(token.MISSION.size() == m_size)
                {
                    c_print("[ WARN]\tDimensione token invariata: ", token.MISSION.size(), red);
                }
            }
            else
            {
                go_home = true;
            }
            need_task = false;
            goal_complete = true;
        }

        // aggiorno la mappa
        init_tw_map();
        // c_print("[DEBUG]\tinit_tw_map() terminato\n\tAggiornamento mappa archi...", yellow);
        for (auto i=0; i<TEAM_SIZE; i++)
        {
            int dst = (int) token.NEXT_VERTEX[i];
            int src = (int) token.CURR_VERTEX[i];
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
                token_weight_map[dst][src] += sec_diff * 3;
                // sfavorisco tutti gli archi che entrano nella mia destinazione
                // dovrebbe prevenire gli scontri agli incroci dove due robot
                // arrivano da nodi diversi
                for( int j=0; j<dimension; j++)
                {
                    if(j != src)
                    {
                        token_weight_map[j][dst] += sec_diff * 2;
                    }
                }
            }
        }

        // metto nel token quale arco sto occupando
        // se sono ancora in attesa di partire inserisco l'arco di inizializzazione
        if(!init_wait_done)
        {
            token.CURR_VERTEX[ID_ROBOT] = current_vertex;
            token.NEXT_VERTEX[ID_ROBOT] = init_next_vertex;
        }
        else
        {
            token.CURR_VERTEX[ID_ROBOT] = current_vertex;
            token.NEXT_VERTEX[ID_ROBOT] = next_vertex;
        }

        // controllo interferenza



        interference = check_interference_token(token);

        if (interference)
        c_print("Robot in interferenza: ",ID_ROBOT,red,P);
    }

        

    usleep(30000);
    token_pub.publish(token);
    ros::spinOnce();
} // token_callback()

// ciao
// ehilà

} //namespace tpagent