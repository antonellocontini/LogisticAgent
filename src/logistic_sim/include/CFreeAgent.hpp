#pragma once
#include "DistrAgent.hpp"

namespace spartagent
{
using namespace distragent;
class SPartAgent : public DistrAgent
{
public:
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
};
} // namespace spartagent

#include "impl/CFreeAgent.i.hpp"