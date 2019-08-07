#pragma once
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <algorithm>

#include <color_cout.hpp>

#include "PatrolAgent.hpp"
#include "config.hpp"

namespace logicagent
{
    using namespace patrolagent;

    class LogicAgent : public PatrolAgent
    {
    protected:
        // variabili
        // bool getTask;
        int id_vertex;
        uint route_dimension;
        uint *route;

        pthread_mutex_t lock;

        std_msgs::Bool      free;       // falg per far sapere che sono senza un task

        ros::Publisher      pub_task;   // ho bisogno di un task
        ros::Subscriber     sub_route;  // letto il task da eseguire    

    public:
        // LogicAgent(ros::NodeHandle &nh_);
        // ~LogicAgent() {};

        // virtual void    init(int argc, char** argv);
        // virtual void    onGoalComplete();
        virtual int     compute_next_vertex();
        virtual void    send_results();
        virtual void    receive_results();

        // int get_Logic_dimension(const char* logic_file);
        // void get_Logic_route(uint *route, uint dimension, const char* logic_file);

        // void getTask(vector<TaskPlanner::Task> &task);
    };
} // namespace logagent

#include "impl/LogicAgent.i.hpp"