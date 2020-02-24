#pragma once

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread.hpp>
#include "TaskPlanner.hpp"
#include<chrono>

namespace onlinetaskplanner
{

const std::string PS_path = ros::package::getPath("logistic_sim");

class OnlineTaskPlanner : public taskplanner::TaskPlanner
{
public:
    OnlineTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineTaskPlanner");
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    bool check_paths_conflicts(const std::vector<logistic_sim::Path> &paths, bool print = true);

    void allocate_memory() override;

protected:
    ros::Time last_goal_time;
    ros::Duration shutdown_timeout = ros::Duration(5 * 60.0), shutdown_warning = ros::Duration(4 * 60.0);
    bool warning_occured = false;
    logistic_sim::Token::_GOAL_STATUS_type last_goal_status;

    bool allocation_phase = false;
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
};

} // namespace onlinetaskplanner
