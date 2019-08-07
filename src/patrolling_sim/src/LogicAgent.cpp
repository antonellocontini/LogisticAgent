#include "LogicAgent.hpp"

using namespace logicagent;
/* 
void LogicAgent::getTask(vector<TaskPlanner::Task> &tasks)
{
    int tmp;
    int count = 0;

    for (vector<TaskPlanner::Task>::iterator it = tasks.begin(); it != tasks.end(); ++it)
    {
        if (!it->take)
        {
            it->take = true;
            route_dimension = it->dimension;
            route = new uint[route_dimension];

            cout <<" Route:\n";
            for (auto i = 0; i < route_dimension; i++)
            {
                route[i] = it->route[i];
                c_print(" ",route[i], red);
            }
            cout<<"\n";
        }
        // tasks.pop_back();
    } 
}
*/