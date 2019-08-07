#pragma once

namespace logicagent
{

// LogicAgent::LogicAgent(ros::NodeHandle &nh_)
// {
//     pub_task    = nh_.advertise<std_msgs::Bool>("task_planner/needtask",1);
//     // init(argc, agrv);
// }

// void LogicAgent::init(int argc, char** argv)
// {
//     pub_task = nh.advertise<std_msgs::Bool>("task_planner/needtask",1); 
//     c_print("# INIT",red);

//     PatrolAgent::init(argc,argv);
   
//     free.data = true;

//     pub_task.publish(free);

//     c_print("# Pub flag!",red);
// }

// void LogicAgent::onGoalComplete()
// {
//     // 
// }

int LogicAgent::compute_next_vertex()
{
    int tmp = 0;
    tmp = route[id_vertex];
    id_vertex++;
    if (id_vertex >= route_dimension)
        id_vertex = 1;
    return tmp;
}

void LogicAgent::send_results()
{
    cout << "SEND_RESULTS!\n";
}

void LogicAgent::receive_results()
{
    cout << "RECEIVE_RESULTS!\n";
}

} // namespace logicagent