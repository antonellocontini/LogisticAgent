#pragma once
#include "DistrAgent.hpp"
#include "logistic_sim/RobotReady.h"

namespace constagent
{
using namespace distragent;
class ConstAgent : public DistrAgent
{
public:
    void run() override;
    std::pair<int,int> check_interference_token(logistic_sim::Token &token) override;
    void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path) override;
};
} // namespace constagent

#include "impl/ConstAgent.i.hpp"