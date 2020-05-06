#include "OnlineDCOPTaskPlanner.hpp"

namespace onlinedcoptaskplanner
{

OnlineDCOPTaskPlanner::OnlineDCOPTaskPlanner(ros::NodeHandle &nh_, const std::string &name)
  : OnlineTaskPlanner(nh_, name)
{

}


void OnlineDCOPTaskPlanner::token_callback(const logistic_sim::TokenConstPtr &msg)
{
  ROS_INFO_STREAM("TODO DCOP TASK PLANNER");
}
} // namespace onlinedcoptaskplanner

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinedcoptaskplanner::OnlineDCOPTaskPlanner ODTP(nh_);
  ODTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ODTP.run();
  return 0;
}