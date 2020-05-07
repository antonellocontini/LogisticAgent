#pragma once
#include "OnlineAgent.hpp"

namespace onlinedcopagent
{

class OnlineDCOPAgent : public onlineagent::OnlineAgent
{
public:
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
protected:
    
};

}