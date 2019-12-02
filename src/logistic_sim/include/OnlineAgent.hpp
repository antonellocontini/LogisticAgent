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
                                                 int start_time = 0);
protected:
    std::vector<std::vector<unsigned int> > map_graph;
};

} // namespace onlineagent