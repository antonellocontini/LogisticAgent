#pragma once

#include "Agent.hpp"
#include "logistic_sim/Token.h"


using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;
namespace logistic_sim
{

inline bool operator==(const logistic_sim::Mission &A, const logistic_sim::Mission &B)
{
    return A.ID == B.ID ? true : false;
}

inline bool operator<(const logistic_sim::Mission &A, const logistic_sim::Mission &B)
{
    return A.V < B.V ? 1 : 0;
}

} // namespace logistic_sim

struct less_V
{
    inline bool operator()(const t_coalition &A, const t_coalition &B)
    {
        return A.second.V < B.second.V;
    }
};

inline bool operator==(const t_coalition &A, const t_coalition &B)
{
    return A.second.ID == B.second.ID ? 1 : 0;
}

inline bool operator<(const t_coalition &A, const t_coalition &B)
{
    return A.second.V < B.second.V ? 1 : 0;
}

namespace distragent
{
using namespace agent;

class DistrAgent : public Agent
{
protected:
    int t_interference = 0;
    int id_interference = -1;
    ros::Publisher token_pub;
    ros::Subscriber token_sub;

    bool need_task = true;
    bool reached_pickup, go_home = false;
    bool reached_home = false;
    uint tmp_CAPACITY = 0;

    bool init_wait_done = false;

    uint init_next_vertex;
    uint src_vertex = 13;

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

    logistic_sim::Mission current_mission;
    std::vector<std::vector<uint>> token_weight_map;

    uint init_wait_time;
    ros::Time init_start_time;
    ros::Time goal_start_time;
    ros::Time mission_start_time;

public:
    void init(int argc, char **argv);
    virtual void run();

    void onGoalComplete();
    int compute_next_vertex();

    void onGoalComplete(logistic_sim::Token &token);
    int compute_next_vertex(logistic_sim::Token &token);
    //
    uint compute_id_path(logistic_sim::Mission &m);
    void compute_travell(uint id_path, logistic_sim::Mission &m);
    int compute_cost_of_route(std::vector<uint> r);
    logistic_sim::Mission coalition_formation(logistic_sim::Token &token);
    //
    virtual void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
    void init_tw_map();
    virtual std::pair<int,int> check_interference_token(logistic_sim::Token &token);
    virtual void token_callback(const logistic_sim::TokenConstPtr &msg);
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) override;

    void print_coalition(const t_coalition &coalition);

    virtual bool go_src();
    virtual bool go_dst();
};
} // namespace distragent

#include "impl/DistrAgent.i.hpp"