#pragma once
#include "CFreeAgent.hpp"

namespace onlineagent
{

class OnlineAgent : public cfreeagent::CFreeAgent
{
public:
    void init(int argc, char **argv) override;
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    std::vector<unsigned int> spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                                 const std::vector<std::vector<unsigned int> > &graph,
                                                 const std::vector<unsigned int> &waypoints,
                                                 int start_time = 0,
                                                 std::vector<unsigned int> *last_leg = nullptr,
                                                 std::vector<unsigned int> *first_leg = nullptr);
protected:
    std::vector<std::vector<unsigned int> > map_graph;

    void token_simple_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
    void token_simple_allocation(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
    void token_simple_planning(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);

    void token_priority_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
    void token_priority_alloc_plan(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
};

} // namespace onlineagent