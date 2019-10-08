#pragma once

#include "boost/filesystem.hpp"
#include "algorithms.hpp"

namespace taskplanner
{

ostream &operator<<(ostream &os, const MonitorData &md)
{
    os << md.tot_distance << "," << md.interference_num << "," << md.completed_missions << "," << md.completed_tasks << "," << md.total_time;
    return os;
}

ostream &operator<<(ostream &os, const vector<MonitorData> &v)
{
    os << "ID_ROBOT,TOT_DISTANCE,INTERFERENCE_NUM,COMPLETED_MISSIONS,COMPLETED_TASKS,TOTAL_TIME" << endl;
    for (int i = 0; i < v.size(); i++)
    {
        os << i << "," << v[i] << endl;
    }
    return os;
}

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_, const std::string &name) : name(name)
{
    sub_token = nh_.subscribe("token", 1, &TaskPlanner::token_Callback, this);
    pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

    nh_.setParam("/simulation_running", "true");
}

void TaskPlanner::init(int argc, char **argv)
{
    srand(time(NULL));
    chdir(PS_path.c_str());
    mapname = string(argv[1]);
    string graph_file = "maps/" + mapname + "/" + mapname + ".graph";
    dimension = GetGraphDimension(graph_file.c_str());
    vertex_web = new vertex[dimension];
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());
    uint nedges = GetNumberEdges(vertex_web, dimension);
    printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);

    TEAM_SIZE = atoi(argv[3]);
    ALGORITHM = argv[2];

    GENERATION = argv[4];

    TEAM_CAPACITY = atoi(argv[5]);

    src_vertex = map_src[mapname];
    dst_vertex = map_dsts[mapname];

    for(uint dst : dst_vertex)
    {
        int result[100];
        uint result_size;
        dijkstra(src_vertex, dst, result, result_size, vertex_web, dimension);
        paths.push_back(std::vector<uint>(result, result + result_size));
    }

    // avvio servizio
    robots_ready_status = std::vector<bool>(TEAM_SIZE);
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("robot_ready", &TaskPlanner::robot_ready, this);
    ros::spinOnce();

    missions_generator(GENERATION);

    // test non uniform sp grid
    // logistic_sim::Mission m;
    // m.DSTS = {16,16};
    // m.DEMANDS = {1,2};
    // m.ITEM = {0,0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {17,17};
    // m.DEMANDS = {1,1};
    // m.ITEM = {0,0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18,18};
    // m.DEMANDS = {1,2};
    // m.ITEM = {0,0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {17};
    // m.DEMANDS = {2};
    // m.ITEM = {0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18,18};
    // m.DEMANDS = {2,1};
    // m.ITEM = {0,0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {17};
    // m.DEMANDS = {3};
    // m.ITEM = {0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18};
    // m.DEMANDS = {3};
    // m.ITEM = {0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18};
    // m.DEMANDS = {3};
    // m.ITEM = {0};
    // missions.push_back(logistic_sim::Mission(m));

    // // taskset di test
    // logistic_sim::Mission m;
    // m.DSTS = {18,28};
    // m.DEMANDS = {1,2};
    // m.ITEM = {0,0};
    // m.TOT_DEMAND = 3;
    // missions.push_back(logistic_sim::Mission(m));
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18};
    // m.DEMANDS = {3};
    // m.ITEM = {0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18,23};
    // m.DEMANDS = {1,2};
    // m.ITEM = {0,0};
    // missions.push_back(logistic_sim::Mission(m));
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {23};
    // m.DEMANDS = {3};
    // m.ITEM = {0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {28};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18};
    // missions.push_back(logistic_sim::Mission(m));
    // // nuovi task
    // m.DSTS = {28,18};
    // m.DEMANDS = {2,1};
    // m.ITEM = {1,0};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {28,18,23};
    // m.DEMANDS = {1,1,1};
    // m.ITEM = {0,1,2};
    // missions.push_back(logistic_sim::Mission(m));
    // m.DSTS = {18,28};
    // m.DEMANDS = {2,1};
    // m.ITEM = {1,0};
    // missions.push_back(logistic_sim::Mission(m));

    set_partition();

    c_print("TEAM: ", TEAM_SIZE, " nTask: ", nTask, magenta);

    // aspetto che arrivino gli agenti
    while (robots_ready_count < TEAM_SIZE)
    {
        ros::Duration(1, 0).sleep();
        ros::spinOnce();
    }
    // sleep(10);

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

int TaskPlanner::compute_cost_of_route(std::vector<uint> &route)
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


logistic_sim::Mission TaskPlanner::create_mission(uint type, int id)
{
    static uint d[3] = {0, 0, 0};
    logistic_sim::Mission m;
    m.PICKUP = false;
    m.ID = id;
    id++;
    m.PRIORITY = 0;
    m.ITEM.push_back(type);

    copy(std::begin(paths[type]), std::end(paths[type]), back_inserter(m.ROUTE));
    m.DSTS.push_back(dst_vertex[type]);
    m.TOT_DEMAND = (d[type] % 3) + 1;
    m.DEMANDS.push_back((d[type] % 3) + 1);
    d[type]++;
    m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
    m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

    return m;
}

void TaskPlanner::u_missions_generator()
{
    int size = 4;
    int size_2 = 3;
    int d = 1;

    static int id = 0;
    for (auto i = 0; i < size; i++)
    {
        // c_print("oh");
        for (auto j = 0; j < size_2; j++)
        {
            // c_print("eh");
            logistic_sim::Mission m;
            m.PICKUP = false;
            m.ID = id;
            id++;
            m.PRIORITY = 0;
            m.ITEM.push_back(i % 3);
            // c_print("ah");
            copy(std::begin(paths[i % 3]), std::end(paths[i % 3]), back_inserter(m.ROUTE));
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

} 


void TaskPlanner::nu_missions_generator()
{
    uint t1_size = 2; 
    uint t2_size = 4;  
    uint t3_size = 6; 
    static int id = 0;
    while (t1_size > 0 || t2_size > 0 || t3_size > 0)
    {
        if (t1_size > 0)
        {
            logistic_sim::Mission m = create_mission(0, id);
            missions.push_back(m);
            t1_size--;
            id++;
            std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD: " << m.DEMANDS[0] << "\tITEM: " << m.ITEM[0] << "\n";
        }
        if (t2_size > 0)
        {
            logistic_sim::Mission m = create_mission(1, id);
            missions.push_back(m);
            t2_size--;
            id++;
            std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD: " << m.DEMANDS[0] << "\tITEM: " << m.ITEM[0] << "\n";
        }
        if (t3_size > 0)
        {
            logistic_sim::Mission m = create_mission(2, id);
            missions.push_back(m);
            t3_size--;
            id++;
            std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD: " << m.DEMANDS[0] << "\tITEM: " << m.ITEM[0] << "\n";
        }
    }

    nTask = missions.size();

    for (auto i = 0; i < missions.size(); i++)
    {
        cout << missions[i].TOT_DEMAND << " \n";
    }
}

void TaskPlanner::missions_generator(std::string &type_gen)
{
    if (type_gen == "uniform")
    {
        u_missions_generator();
    }
    else if(type_gen == "non-uniform")
    {
        nu_missions_generator();
    }
    else
    {
        c_print("ERR GEN TYPE", red, P);
    }
}


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
        for(auto m : missions)
        {
            for (auto dst : m.DSTS )
            {
                std::cout << dst << " ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
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
            for (int i = 0; i < num_robots; i++)
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
        for (int i = 0; i < last_interf_count.size(); i++)
        {
            if (token.INTERFERENCE_COUNTER[i] > last_interf_count[i])
            {
                last_interf_count[i] = token.INTERFERENCE_COUNTER[i];
                c_print("Interferenza rilevata dal robot ", i, "!", red);
            }
        }

        // aggiorno la mia struttura con i dati del token
        for (int i = 0; i < num_robots; i++)
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
            conf_dir_name << "results/" << name << "_" << ALGORITHM << "_" << GENERATION << "_teamsize" << num_robots << "capacity" << CAPACITY[0] << "_" << mapname;
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
                filename.str(""); // cancella la stringa
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

bool TaskPlanner::robot_ready(logistic_sim::RobotReady::Request &req,
                              logistic_sim::RobotReady::Response &res)
{
    uint id_robot = req.ID_ROBOT;
    c_print("Robot ", id_robot, " is ready", green, P);
    if (!robots_ready_status[id_robot])
    {
        robots_ready_status[id_robot] = true;
        robots_ready_count++;
    }

    return true;
}

} // namespace taskplanner
