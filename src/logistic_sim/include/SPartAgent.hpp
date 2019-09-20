#pragma once
#include "DistrAgent.hpp"

#include "partition.hpp"

namespace spartagent
{
using namespace distragent;
class SPartAgent : public DistrAgent
{
public:
    void run() override;
    logistic_sim::Mission coalition_formation(logistic_sim::Token &token) override;
    logistic_sim::Mission set_partition_tasks(logistic_sim::Token &token);
};
} // namespace spartagent

#include "impl/SPartAgent.i.hpp"