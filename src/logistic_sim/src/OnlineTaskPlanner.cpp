#include "OnlineTaskPlanner.hpp"

namespace onlinetaskplanner
{

OnlineTaskPlanner::OnlineTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
{
    sub_token = nh_.subscribe("token", 1, &OnlineTaskPlanner::token_callback, this);
    pub_token = nh_.advertise<logistic_sim::Token>("token", 1);

    nh_.setParam("/simulation_running", "true");
}

void OnlineTaskPlanner::init(int argc, char **argv)
{

}

void OnlineTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{

}

} // namespace onlinetaskplanner


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_planner");
    ros::NodeHandle nh_;
    onlinetaskplanner::OnlineTaskPlanner OTP(nh_);
    OTP.init(argc, argv);
    c_print("inizializzazione finita!", green);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}