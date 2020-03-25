#pragma once
#include "DistrAgent.hpp"

namespace onlinecentralizedagent
{


class OnlineCentralizedAgent : public distragent::DistrAgent
{
public:
  void init(int argc, char **argv) override;
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

protected:
  bool still = true;

  void token_coordination(const logistic_sim::TokenConstPtr &msg, logistic_sim::Token &token);
};

}  // namespace onlinecentralizedagent