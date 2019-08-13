#include "LogicAgent.hpp"

using namespace logicagent;

void LogicAgent::init(int argc, char** argv)
{
    Agent::init(argc, argv);

    ros::NodeHandle nh;

    pub_task_planner = nh.advertise<task_planner::Task>("",1);
    sub_task_planner = nh.subscribe<task_planner::Task>("",100,boost::bind(&LogicAgent::receive_Task_Callback, this, _1));
    // possibili variabili da settare
}

void LogicAgent::run()
{

}

void LogicAgent::onGoalComplete()
{

}

int LogicAgent::compute_next_vertex()
{

}

int main(int argc, char *argv[])
{
    logicagent::LogicAgent TPA;
    TPA.init(argc, argv);
    c_print("@ Inizializzazione finita!",green);
    sleep(3);
    TPA.run();
    return 0;
}