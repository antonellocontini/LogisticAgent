#include "TaskPlanner.hpp"

using namespace taskplanner;

void TaskPlanner::set_partition()
{
  c_print("set_partition on TP Base", red);
}

std::vector<logistic_sim::Path> TaskPlanner::path_partition()
{
  c_print("path_partition on TP Base", red);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_planner");

  ros::NodeHandle nh_;  // con ~ avremmo il prefisso sui topic

  taskplanner::TaskPlanner TP(nh_);

  TP.init(argc, argv);

  c_print("inizializzazione finita!", green);

  ros::AsyncSpinner spinner(2);

  spinner.start();

  ros::waitForShutdown();

  return 0;
}
