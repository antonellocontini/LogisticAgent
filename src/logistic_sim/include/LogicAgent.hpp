#pragma once
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <algorithm>

#include <color_cout.hpp>

#include "Agent.hpp"
#include "config.hpp"
#include <task_planner/Task.h>


// LogicAgent Centralizzato contatta il TaskPlanner 

namespace logicagent
{
    using namespace agent;

    class LogicAgent : public Agent
    {
    protected:
        // pub/sub con il task_planner
        ros::Subscriber sub_task_planner;
        ros::Publisher  pub_task_planner;  

    public:

        virtual void init(int argc, char** argv);
        virtual void run();
        virtual void onGoalComplete();
        virtual int compute_next_vertex();

        void request_Task();
        void receive_Task_Callback(const task_planner::TaskConstPtr &msg);
    };
} // namespace logagent

#include "impl/LogicAgent.i.hpp"