#pragma once

#include "Agent.hpp"
#include "logistic_sim/Token.h"
#include "logistic_sim/RobotReady.h"

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

/**
 * Approccio senza turni con grafo dinamico, non più mantenuto 
 */
class DistrAgent : public Agent
{
protected:
    ros::Publisher token_pub;
    ros::Subscriber token_sub;

    bool reached_home = false;

    uint init_next_vertex;  //TODO: togliere, usato in metodi vecchi
    std::map<std::string, uint> map_src = {{"model6", 13}, {"grid", 7}, {"icelab", 22}, {"icelab_black", 2}, {"model5", 6}};
    std::map<std::string, std::vector<uint>> map_dsts = {{"model6", {18, 23, 28}}, {"grid", {16, 17, 18}},
                                                         {"icelab", {10, 13, 16}}, {"icelab_black", {26, 33, 42}},
                                                         {"model5", {11, 16, 21}}};
    uint src_vertex;
    std::vector<uint> dsts_vertex;

    logistic_sim::Mission current_mission;
    std::vector<std::vector<uint>> token_weight_map;

    uint init_wait_time;
    ros::Time init_start_time;
    ros::Time goal_start_time;
    ros::Time mission_start_time;

public:
    void init(int argc, char **argv) override;
    virtual void run();

    int compute_cost_of_route(std::vector<uint> r);
    
    void init_tw_map();
    virtual void token_callback(const logistic_sim::TokenConstPtr &msg) = 0;
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) override;

    void print_coalition(const t_coalition &coalition);
};
} // namespace distragent

#include "impl/DistrAgent.i.hpp"