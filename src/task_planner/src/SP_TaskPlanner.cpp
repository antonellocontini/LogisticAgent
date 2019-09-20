#include "SP_TaskPlanner.hpp"

using namespace sp_taskplanner;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "task_planner");

    ros::NodeHandle nh_; // con ~ avremmo il prefisso sui topic

    sp_taskplanner::SP_TaskPlanner SPTP(nh_);

    SPTP.init(argc, argv);

    c_print("inizializzazione finita!", green);

    ros::AsyncSpinner spinner(2);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}