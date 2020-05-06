#include "OnlineTaskPlanner.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinetaskplanner::OnlineTaskPlanner OTP(nh_);
  OTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  OTP.run();
  return 0;
}