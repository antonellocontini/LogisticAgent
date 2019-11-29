#pragma once

#include "TaskPlanner.hpp"

namespace onlinetaskplanner
{

class OnlineTaskPlanner : public taskplanner::TaskPlanner
{
public:
    OnlineTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineTaskPlanner");
    void init(int argc, char **argv);
    void run();
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
protected:

};

} // namespace onlinetaskplanner
