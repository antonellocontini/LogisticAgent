#pragma once
#include "DistrAgent.hpp"

namespace cfreeagent
{
using namespace distragent;
class CFreeAgent : public DistrAgent
{
public:
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    void token_dijkstra(uint source, uint destination, std::vector<logistic_sim::Path> &other_paths);
};
} // namespace spartagent

#include "impl/CFreeAgent.i.hpp"