#include "Agent.hpp"
#pragma once
#include "logistic_sim/Token.h"

namespace logistic_sim
{
    bool operator== (const Mission& A, const Mission& B)
    {
        return A.ID == B.ID ? true : false;
    }

    bool operator== (const Task& A, const Task& B)
    {
        return A.ORDER == B.ORDER ? true : false;
    }
}

namespace distragent
{
using namespace agent;

bool cmp_Mission(logistic_sim::Mission A, logistic_sim::Mission B)
{
    return A.ID == B.ID ? 1 : 0;
}


inline bool operator<(const logistic_sim::Mission& A, const logistic_sim::Mission& B)
{
    return A.V < B.V ? 1 : 0;
}

class DistrAgent : public Agent
{
protected:
    ros::Publisher token_pub;
    ros::Subscriber token_sub;

    bool need_task = true;
    bool reached_pickup, go_home = false;
    uint tmp_CAPACITY = 0;

    uint p_11[8] = {6, 7, 9, 12, 11, 10, 8, 5};
    uint p_16[12] = {6, 7, 9, 12, 14, 17, 16, 15, 13, 10, 8, 5};
    uint p_21[16] = {6, 7, 9, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5};

    uint p_11_16[14] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 15, 13, 10, 8, 5};
    uint p_11_21[18] = {6, 7, 9, 12, 11, 12, 14, 17, 19,
                        22, 21, 20, 18, 15, 13, 10, 8, 5};
    uint p_16_21[18] = {6, 7, 9, 12, 14, 17, 16, 17, 19, 22,
                        21, 20, 18, 15, 13, 10, 8, 5};

    uint p_11_16_21[20] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 17,
                           19, 22, 21, 20, 18, 15, 13, 10, 8, 5};
    
    logistic_sim::Task current_task;
    logistic_sim::Mission current_mission;
    std::vector<std::vector<uint>> token_weight_map;

public:

    virtual void init(int argc, char **argv);
    virtual void run();
    virtual void onGoalComplete();
    virtual int compute_next_vertex();
    //
    uint compute_id_path(logistic_sim::Mission &m);
    void compute_travell(uint id_path, logistic_sim::Mission &m);
    int compute_cost_of_route(std::vector<uint> r);
    logistic_sim::Mission coalition_formation(logistic_sim::Token &token);
    //
    void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
    void init_tw_map();
    void token_callback(const logistic_sim::TokenConstPtr &msg);

    virtual bool go_src();
    virtual bool go_dst();
};
} // namespace distragent

#include "impl/DistrAgent.i.hpp"