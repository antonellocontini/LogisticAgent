#pragma once
#include "DistrAgent.hpp"

namespace cfreeagent
{
using namespace distragent;
class CFreeAgent : public DistrAgent
{
protected:
    bool path_calculated = false;
    std::vector<logistic_sim::Mission> missions;
public:
    bool token_check_pt(std::vector<uint> &my_path, std::vector<logistic_sim::Path> &other_paths, uint ID_ROBOT, int * id_vertex_stuck);
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    void token_dijkstra(const std::vector<uint> &waypoints, std::vector<logistic_sim::Path> &other_paths);
    std::vector<unsigned int> spacetime_dijkstra(const std::vector<std::vector<unsigned int> > &other_paths, const std::vector<std::vector<unsigned int> > &graph, unsigned int size, const std::vector<unsigned int> &waypoints);

};
} // namespace spartagent

#include "impl/CFreeAgent.i.hpp"