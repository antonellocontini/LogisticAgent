#include "TaskPlanner.hpp"

using namespace taskplanner;

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
