#include "SP_TaskPlanner.hpp"

namespace sp_taskplanner
{

void SP_TaskPlanner::init(int argc, char **argv)
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

    set_partition();

    // // aspetto che arrivino gli agenti
    // sleep(10);

    // // giro di inizializzazione
    // logistic_sim::Token token;
    // token.ID_SENDER = TASK_PLANNER_ID;
    // token.ID_RECEIVER = 0;
    // token.INIT = true;

    // pub_token.publish(token);
    // ros::spinOnce();

    // sleep(1);

    // c_print("INIT", green);
}

void SP_TaskPlanner::token_Callback(const logistic_sim::TokenConstPtr &msg)
{
    TaskPlanner::token_Callback(msg);
}

} // namespace sp_taskplanner

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_planner");

    ros::NodeHandle nh_; // con ~ avremmo il prefisso sui topic

    sp_taskplanner::SP_TaskPlanner SPTP(nh_);

    SPTP.init(argc, argv);

    c_print("inizializzazione finita!", green);

    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}