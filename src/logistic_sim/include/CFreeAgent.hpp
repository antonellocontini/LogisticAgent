#pragma once
#include "DistrAgent.hpp"

namespace cfreeagent
{
using namespace distragent;
class CFreeAgent : public DistrAgent
{
protected:
    bool path_calculated = false;
    bool reached_home = false;
    std::vector<logistic_sim::Mission> missions;
public:
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    void token_dijkstra(uint source, uint destination, std::vector<logistic_sim::Path> &other_paths);
};
} // namespace spartagent

#include "impl/CFreeAgent.i.hpp"