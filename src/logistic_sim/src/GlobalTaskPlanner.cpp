#include "GlobalTaskPlanner.hpp"

using namespace globaltaskplanner;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_planner");

    ros::NodeHandle nh_; // con ~ avremmo il prefisso sui topic

    globaltaskplanner::GlobalTaskPlanner GTP(nh_);

    GTP.init(argc, argv);

    c_print("initialization completed!", green);

    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}