#pragma once
#include "DistrAgent.hpp"

#include "partition.hpp"

struct Candidate
{
    uint id;
    uint subset;
    std::vector<std::vector<logistic_sim::Mission>> vv_MISSION;
    uint good;
    double V;
};

namespace spartagent
{
using namespace distragent;
class SPartAgent : public DistrAgent
{
public:
    void run() override;
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    logistic_sim::Mission set_partition_tasks(logistic_sim::Token &token);
};
} // namespace spartagent

#include "impl/SPartAgent.i.hpp"