#pragma once
#include "DistrAgent.hpp"

namespace constagent
{
using namespace distragent;
class ConstAgent : public DistrAgent
{
public:
    void run() override;
    std::pair<int,int> check_interference_token(logistic_sim::Token &token) override;
};
} // namespace constagent

#include "impl/ConstAgent.i.hpp"