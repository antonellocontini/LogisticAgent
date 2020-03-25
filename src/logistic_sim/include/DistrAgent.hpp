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

    // TODO: read from parameter file (see task planners)
    std::map<std::string, uint> map_src = {{"model6", 13}, {"grid", 7}, {"icelab", 22}, {"icelab_black", 2}, {"model5", 6}};
    std::map<std::string, std::vector<uint>> map_dsts = {{"model6", {18, 23, 28}}, {"grid", {16, 17, 18}},
                                                         {"icelab", {10, 13, 16}}, {"icelab_black", {26, 33, 42}},
                                                         {"model5", {11, 16, 21}}};
    
    // here are kept pickup and delivery vertices
    uint src_vertex;
    std::vector<uint> dsts_vertex;

public:
    void init(int argc, char **argv) override;
    virtual void run();
    
    virtual void token_callback(const logistic_sim::TokenConstPtr &msg) = 0;
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) override;
};
} // namespace distragent