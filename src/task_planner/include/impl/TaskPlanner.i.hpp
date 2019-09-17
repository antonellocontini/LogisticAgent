#pragma once

#include "boost/filesystem.hpp"

namespace taskplanner
{

ostream& operator<<(ostream &os, const MonitorData &md)
{
    os << md.tot_distance << "," << md.interference_num << "," << md.completed_missions << "," << md.completed_tasks << "," << md.total_time;
    return os;
}

ostream& operator<<(ostream &os, const vector<MonitorData> &v)
{
    os << "ID_ROBOT,TOT_DISTANCE,INTERFERENCE_NUM,COMPLETED_MISSIONS,COMPLETED_TASKS,TOTAL_TIME" << endl;
    for(int i=0; i<v.size(); i++)
    {
        os << i << "," << v[i] << endl;
    }
    return os;
}

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
    sub_token = nh_.subscribe("token", 1, &TaskPlanner::token_Callback, this);
    pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

    nh_.setParam("/simulation_running", "true");
}

void TaskPlanner::init(int argc, char **argv)
{
    srand(time(NULL));
    chdir(PS_path.c_str());
    string mapname = string(argv[1]);
    string graph_file = "maps/" + mapname + "/" + mapname + ".graph";
    uint dimension = GetGraphDimension(graph_file.c_str());
    vertex_web = new vertex[dimension];
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());
    uint nedges = GetNumberEdges(vertex_web, dimension);
    printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);
    TEAM_SIZE = atoi(argv[3]);
    ALGORITHM = argv[2]; 
    
    missions_generator();

    c_print("TEAM: ", TEAM_SIZE, " nTask: ", nTask, magenta);

    // aspetto che arrivino gli agenti
    sleep(10);

    // giro di inizializzazione
    logistic_sim::Token token;
    token.ID_SENDER = TASK_PLANNER_ID;
    token.ID_RECEIVER = 0;
    token.INIT = true;

    pub_token.publish(token);
    ros::spinOnce();

    sleep(1);

    c_print("INIT", green);
}

int TaskPlanner::compute_cost_of_route(std::vector<uint> route)
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

void TaskPlanner::missions_generator()
{
    int size = 4;
    int size_2 = 3;
    int d = 1;

    static int id = 0;
    for (auto i = 0; i < size; i++)
    {
        for (auto j = 0; j < size_2; j++)
        {

            logistic_sim::Mission m;
            m.PICKUP = false;
            m.ID = id;
            id++;
            m.PRIORITY = 0;
            m.ITEM.push_back(i % 3);

            switch (i % 3)
            {
            case 0:
            {
                copy(std::begin(p_11), std::end(p_11), back_inserter(m.ROUTE));
                break;
            }
            case 1:
            {
                copy(std::begin(p_16), std::end(p_16), back_inserter(m.ROUTE));
                break;
            }
            case 2:
            {
                copy(std::begin(p_21), std::end(p_21), back_inserter(m.ROUTE));
                break;
            }
            default:
            {
                c_print("[DEBUG]", yellow, P);
                break;
            }
            }
            m.DSTS.push_back(dst_vertex[i % 3]);
            m.TOT_DEMAND = (j % 3) + 1;
            m.DEMANDS.push_back((j % 3) + 1);
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
            missions.push_back(m);

            std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD:" << m.DEMANDS[0] << "\n";
        }
    }

    nTask = missions.size();

    for (auto i = 0; i < missions.size(); i++)
    {
        cout << missions[i].TOT_DEMAND << " \n";
    }

} // namespace taskplanner

void TaskPlanner::token_Callback(const logistic_sim::TokenConstPtr &msg)
{
    static int last_mission_size = 0;
    static vector<int> last_interf_count;
    if (msg->ID_RECEIVER != TASK_PLANNER_ID)
        return;

    logistic_sim::Token token;
    token = *msg;
    token.ID_SENDER = TASK_PLANNER_ID;
    token.ID_RECEIVER = 0;
    if (msg->INIT)
    {
        token.HEADER.seq = 1;
        CAPACITY = msg->CAPACITY;
        token.INIT = false;
        token.MISSION = missions;
        token.END_SIMULATION = false;
        start_time = ros::Time::now();
        last_mission_size = missions.size();
    }
    else
    {
        token.HEADER.seq += 1;
        // calcolo quanti robot ci sono e preparo la struttura dati
        if (first_round)
        {
            // per come i robot sono ordinati l'ID del sender equivale al numero di robot meno 1
            num_robots = msg->ID_SENDER + 1;
            for(int i=0; i<num_robots; i++)
            {
                MonitorData data;
                robots_data.push_back(data);
            }

            last_interf_count = vector<int>(num_robots, 0);

            first_round = false;
        }

        // stampa task rimanenti
        if (token.MISSION.size() < last_mission_size)
        {
            last_mission_size = token.MISSION.size();
            c_print("Task rimanenti: ", last_mission_size, green);
        }

        // stampa quando avviene interferenza
        for(int i=0; i<last_interf_count.size(); i++)
        {
            if (token.INTERFERENCE_COUNTER[i] > last_interf_count[i])
            {
                last_interf_count[i] = token.INTERFERENCE_COUNTER[i];
                c_print("Interferenza rilevata dal robot ", i, "!", red);
            }
        }

        // aggiorno la mia struttura con i dati del token
        for (int i=0; i<num_robots; i++)
        {
            robots_data[i].interference_num = token.INTERFERENCE_COUNTER[i];
            robots_data[i].completed_missions = token.MISSIONS_COMPLETED[i];
            robots_data[i].completed_tasks = token.TASKS_COMPLETED[i];
            robots_data[i].tot_distance = token.TOTAL_DISTANCE[i];
            robots_data[i].total_time = (ros::Time::now() - start_time).sec;
        }

        if (token.INIT_POS.empty())
        {
            boost::filesystem::path results_directory("results");
            if (!boost::filesystem::exists(results_directory))
            {
                boost::filesystem::create_directory(results_directory);
            }

            std::stringstream conf_dir_name;
            conf_dir_name << "results/" << ALGORITHM << "_teamsize" << num_robots << "capacity" << CAPACITY[0];  
            boost::filesystem::path conf_directory(conf_dir_name.str());
            if (!boost::filesystem::exists(conf_directory))
            {
                boost::filesystem::create_directory(conf_directory);
            }

            int run_number = 1;
            std::stringstream filename;
            std::ifstream check_new;
            // loop per controllare se il file già esiste
            do
            {
                filename.str("");   // cancella la stringa
                filename << conf_dir_name.str() << "/" << run_number << ".csv";
                check_new = std::ifstream(filename.str());
                run_number++;
            } while (check_new);
            check_new.close();
            
            
            ofstream stats(filename.str());
            stats << robots_data;
            stats.close();
            ros::NodeHandle nh;
            nh.setParam("/simulation_running", "false");
            token.END_SIMULATION = true;
        }
    }

    pub_token.publish(token);
    ros::spinOnce();
    if (token.END_SIMULATION)
    {
        ros::shutdown();
        system("./stop_experiment.sh");
    }
}

} // namespace taskplanner
