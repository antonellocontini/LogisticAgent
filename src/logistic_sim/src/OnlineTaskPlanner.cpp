#include "OnlineTaskPlanner.hpp"

namespace onlinetaskplanner
{

OnlineTaskPlanner::OnlineTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
{
    sub_token = nh_.subscribe("token", 1, &OnlineTaskPlanner::token_callback, this);
    pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

    nh_.setParam("/simulation_running", "true");
}

void OnlineTaskPlanner::init(int argc, char **argv)
{
    srand(time(NULL));
    chdir(PS_path.c_str());
    mapname = string(argv[1]);
    std::string graph_file = "maps/" + mapname + "/" + mapname + ".graph";
    dimension = GetGraphDimension(graph_file.c_str());
    vertex_web = new vertex[dimension];
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());
    uint nedges = GetNumberEdges(vertex_web, dimension);
    printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);

    ALGORITHM = argv[2];
    TEAM_SIZE = atoi(argv[3]);
    GENERATION = argv[4];
    TEAM_CAPACITY = atoi(argv[5]);

    // PARAMETRO TASKSET
    task_set_file = argv[6];

    src_vertex = map_src[mapname];
    dst_vertex = map_dsts[mapname];

    // costruisce percorsi per euristica
    for (uint dst : dst_vertex)
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

    allocate_memory();
    missions_generator(GENERATION);

    // print missions
    std::cout << "Single item task-set:\n";
    for (const logistic_sim::Mission &m : missions)
    {
        std::cout << "ID: " << m.ID << "\n";
        std::cout << "DEMANDS:\n";
        for (auto v : m.DEMANDS)
        {
            std::cout << v << " ";
        }
        std::cout << "DSTS:\n";
        for (auto v : m.DSTS)
        {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

    logistic_sim::Token token;

    // giro di inizializzazione
    token.ID_SENDER = TASK_PLANNER_ID;
    token.ID_RECEIVER = 0;
    token.INIT = true;

    pub_token.publish(token);
    ros::spinOnce();

    sleep(1);

    c_print("INIT", green);
}

void OnlineTaskPlanner::run()
{
    while(ros::ok())
    {
        /*
         * in questo loop creo le tranche di task multi-item
         * da inserire nel token, ogni task dovrebbe indicare
         * a quale tranche appartiene in modo da non intervallarli
         */
    }
}

void OnlineTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{
    /*
     * qua gestisco la raccolta delle statistiche (come sempre)
     * inoltre devo gestire l'inserimento di nuovi task multi-item
     * quando sono pronti (necessario meccanismo di sincronizzazione)
     * 
     * una volta inseriti i nuovi task nel token si abilita una flag
     * che indica agli agenti che nuovi task sono stati inseriti e che
     * devono allocare e pianificare
     * 
     * per ora facciamo tornare gli agenti a casa alla fine di ogni
     * tranche quindi non ci dovrebbero essere problemi con la pianificazione
     * 
     * deve sempre valere il solito meccanismo di rimozione robot in
     * caso di fallimento
     * 
     * cosa nuova con una nuova tranche i robot spenti devono essere
     * riabilitati
     */
}

} // namespace onlinetaskplanner

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle nh_;
    onlinetaskplanner::OnlineTaskPlanner OTP(nh_);
    OTP.init(argc, argv);
    c_print("inizializzazione finita!", green);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    OTP.run();
    return 0;
}