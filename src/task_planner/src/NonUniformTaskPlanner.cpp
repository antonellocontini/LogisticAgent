#include "NonUniformTaskPlanner.hpp"

namespace nonuniformtaskplanner
{

logistic_sim::Mission NonUniformTaskPlanner::create_mission(uint type, int id)
{
    static uint d[3] = {0, 0, 0};
    logistic_sim::Mission m;
    m.PICKUP = false;
    m.ID = id;
    id++;
    m.PRIORITY = 0;
    m.ITEM.push_back(type);

    switch (type)
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
    m.DSTS.push_back(dst_vertex[type]);
    m.TOT_DEMAND = (d[type] % 3) + 1;
    m.DEMANDS.push_back((d[type] % 3) + 1);
    d[type]++;
    m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
    m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;

    return m;
}

void NonUniformTaskPlanner::missions_generator()
{

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
            std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD:" << m.DEMANDS[0] << "\tITEM: " << m.ITEM[0] << "\n";
        }
        if (t3_size > 0)
        {
            logistic_sim::Mission m = create_mission(2, id);
            missions.push_back(m);
            t3_size--;
            id++;
            std::cout << "\tid: " << m.ID << "\tTOT_D: " << m.TOT_DEMAND << "\tdst: " << m.DSTS[0] << "\tD:" << m.DEMANDS[0] << "\tITEM: " << m.ITEM[0] << "\n";
        }
    }

    nTask = missions.size();

    for (auto i = 0; i < missions.size(); i++)
    {
        cout << missions[i].TOT_DEMAND << " \n";
    }
}

void NonUniformTaskPlanner::token_Callback( const logistic_sim::TokenConstPtr &msg)
{
    TaskPlanner::token_Callback(msg);
}
    
} // namespace nonuniformtaskplanner

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "task_planner");

    ros::NodeHandle nh_;  // con ~ avremmo il prefisso sui topic

    nonuniformtaskplanner::NonUniformTaskPlanner TP(nh_);

    TP.init(argc, argv);

    c_print("inizializzazione finita!", green);

    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}